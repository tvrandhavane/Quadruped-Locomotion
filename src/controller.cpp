#include "controller.h"

controller::controller(ODEBodies * body_bag, float * root_position){
	time = 0;
	T = 100;

	this->root_position = root_position;

	swingStart[0] = 1;
	swingEnd[0] = 50;
	swingStart[1] = 1;
	swingEnd[1] = 50;
	swingStart[2] = 0;
	swingEnd[2] = 0;
	swingStart[3] = 0;
	swingEnd[3] = 0;

	shoulderHeight = 470;		//in 0.1cm
    hipHeight = 450;

    current_velocity[0] = 2000;		//in 0.1cm/s
    current_velocity[1] = 0;
    current_velocity[2] = 0;
    desired_velocity[0] = 4000;
    desired_velocity[1] = 0;
    desired_velocity[2] = 0;

    lfHeight[0] = shoulderHeight;
    lfHeight[1] = shoulderHeight;
    lfHeight[2] = hipHeight;
    lfHeight[3] = hipHeight;

    this->body_bag = body_bag;

    for(int i = 0; i< 4; i++){
    	prev_stepping_location[i][0] = 0.0;
    	prev_stepping_location[i][1] = 0.0;
    	prev_stepping_location[i][2] = 0.0;
    	next_stepping_location[i][0] = 0.0;
    	next_stepping_location[i][1] = 0.0;
    	next_stepping_location[i][2] = 0.0;
    	current_foot_location[i][0] = 0.0;
    	current_foot_location[i][1] = 0.0;
    	current_foot_location[i][2] = 0.0;
    }
}

void controller::takeStep(){
	time = (time + 1);
	int phase = time%T;

	cout << "Time = " << time << endl;

	//for each leg
	for(int i = 0; i < 4; i++){
		legController(i, phase);
	}



	vector<float> lengths = body_bag->getLengths(0);
    vector<float> angles = body_bag->getAngles(0);
    vector<float> endEffector(3);
    const dReal *endEffectorPos = dBodyGetPosition(body_bag->getFootLinkBody(0, 3));
    endEffector[0] = -(endEffectorPos[0]-body_bag->getFootLinkLength(0, 3)/2);
    endEffector[1] = endEffectorPos[1];
    endEffector[2] = endEffectorPos[2]-root_position[2];

    QSMatrix<float> * matrix = new QSMatrix<float> (4, 4, 0.0);
    for(int i = 0; i < matrix->get_rows(); i++){
        (*matrix)(i,i) = 1;
    }
    (*matrix)(0, 3) = root_position[0];
    (*matrix)(1, 3) = root_position[1];
    (*matrix)(2, 3) = root_position[2] + 120.0;

    QSMatrix<float> * matrix2 = new QSMatrix<float> (4, 4, 0.0);
    float theta = -(90*M_PI)/180;
    (*matrix2)(0, 0) = cos(theta);
    (*matrix2)(0, 1) = -sin(theta);
    (*matrix2)(1, 0) = sin(theta);
    (*matrix2)(1, 1) = cos(theta);
    (*matrix2)(2, 2) = 1;
    (*matrix2)(3, 3) = 1;

    QSMatrix<float> transformationMatrix = (*matrix) * (*matrix2);
    QSMatrix<float> axis(4, 1, 0.0);
    
    axis(2, 0) = 1;
    axis(3, 0) = 1;
    applyIK(lengths, angles, endEffector, transformationMatrix, axis);

    gravityCompensation();
}

void controller::legController(int leg_id, int phase){
	if(swingStart[leg_id] == phase){		
		computePlacementLocation(leg_id, lfHeight[0]);
		setFootLocation(leg_id, phase);
	}
	else if(isInSwing(leg_id)){
		setFootLocation(leg_id, phase);
		float * targetPosition = getTargetPosition(leg_id);
		//Apply IK and get change in angles
		//Use PD controllers to get torque
	}
	else{
		stanceLegTreatment(leg_id);
	}
}

float * controller::getTargetPosition(int leg_id){
	//translation
	Eigen::MatrixXf mt = Eigen::MatrixXf::Identity(4, 4);
	mt(0, 3) = body_bag->getFootLinkLength(leg_id, 3);

	//rotation
	Eigen::MatrixXf mr = Eigen::MatrixXf::Identity(4, 4);
	const dReal *rotation_matrix_ode = dBodyGetRotation(body_bag->getFootLinkBody(leg_id, 3));
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 4; j++){
			mr(i, j) = rotation_matrix_ode[i*4+j];
		}
	}

	mr(3, 0) = 0;
	mr(3, 1) = 0;
	mr(3, 2) = 0;
	mr(3, 3) = 1;

	Eigen::MatrixXf transformationMatrix = mr*mt;
	
	cout << endl;
	cout << "Here is the matrix transformationMatrix:" << endl << transformationMatrix << endl;
	cout << "Its inverse is:" << endl << transformationMatrix.inverse() << endl;
	cout << "The product of the two (supposedly the identity) is:" << endl << transformationMatrix.inverse()*transformationMatrix << endl;
	//current_foot_location
	//Subtract (vector) the last two links of the leg to get the target end effector position 
}

void controller::applyIK(vector<float> lengths, vector<float> angles, vector<float> endEffector, QSMatrix<float> transformationMatrix, QSMatrix<float> axis){
	inverseKinematics * IKSolver = new inverseKinematics(lengths, angles, endEffector, transformationMatrix, axis);
	delete IKSolver;
}

void controller::gravityCompensation(){
	dBodyAddForce(body_bag->getBackLink1Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getBackLink2Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getBackLink3Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getBackLink4Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getBackLink5Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getBackLink6Body(), 0.0, 98.1, 0.0);
    //Neck and head
    dBodyAddForce(body_bag->getNnhLink1Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getNnhLink2Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getNnhLink3Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getNnhLink4Body(), 0.0, 98.1, 0.0);
    //Tail
    dBodyAddForce(body_bag->getTailLink1Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getTailLink2Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getTailLink3Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getTailLink4Body(), 0.0, 98.1, 0.0);
    //Front left foot
    dBodyAddForce(body_bag->getFootLinkBody(0,0), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getFootLinkBody(0,1), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getFootLinkBody(0,2), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getFootLinkBody(0,3), 0.0, 98.1, 0.0);
    //Front right foot    
    dBodyAddForce(body_bag->getFootLinkBody(1,0), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getFootLinkBody(1,1), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getFootLinkBody(1,2), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getFootLinkBody(1,3), 0.0, 98.1, 0.0);
    //Back left foot
    dBodyAddForce(body_bag->getFootLinkBody(2,0), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getFootLinkBody(2,1), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getFootLinkBody(2,2), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getFootLinkBody(2,3), 0.0, 98.1, 0.0);
    //Back right foot    
    dBodyAddForce(body_bag->getFootLinkBody(3,0), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getFootLinkBody(3,1), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getFootLinkBody(3,2), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getFootLinkBody(3,3), 0.0, 98.1, 0.0);
}

float controller::computeSwingPhase(int leg_id, int phase){
	float swingPhaseLength, phaseLength;
	if(swingStart[leg_id] < swingEnd[leg_id]){
		swingPhaseLength = swingEnd[leg_id] - swingStart[leg_id];
		if(phase < swingEnd[leg_id]){
			if(phase > swingStart[leg_id]){
				phaseLength = phase - swingStart[leg_id];
			}
		}
	}
	else{
		swingPhaseLength = T - swingStart[leg_id] + swingEnd[leg_id];
		if(phase < swingEnd[leg_id]){
			phaseLength = T - swingStart[leg_id] + phase;
		}
		else if(phase > swingStart[leg_id]){
			phaseLength = phase - swingStart[leg_id];
		}
	}

	return phaseLength/swingPhaseLength;
}

void controller::setFootLocation(int leg_id, int phase){
	float swing_phase = computeSwingPhase(leg_id, phase);

	current_foot_location[leg_id][0] = (1 - swing_phase)*prev_stepping_location[leg_id][0] + swing_phase*next_stepping_location[leg_id][0];
	current_foot_location[leg_id][1] = (1 - swing_phase)*prev_stepping_location[leg_id][1] + swing_phase*next_stepping_location[leg_id][1];
	current_foot_location[leg_id][2] = (1 - swing_phase)*prev_stepping_location[leg_id][2] + swing_phase*next_stepping_location[leg_id][2];
}

void controller::stanceLegTreatment(int leg_id){
	cout << "stance leg treatment" << endl;
}

bool controller::isInSwing(int leg_id){
	int phase = time%T;
	if(swingStart[leg_id] < swingEnd[leg_id]){
		if(phase >= swingStart[leg_id] && phase < swingEnd[leg_id]){
			return true;
		}
	}
	else if(swingStart[leg_id] == swingEnd[leg_id]){
		return false;
	}
	else{
		if(phase >= swingStart[leg_id]){
			return true;
		}
		else if(phase < swingEnd[leg_id]){
			return true;
		}
	}
	return false;
}

void controller::computePlacementLocation(int leg_id, float h){
	cout << "compute placement location " << endl;
	float df[3];

	float velocitySqr = desired_velocity[0]*desired_velocity[0];
	velocitySqr += desired_velocity[1]*desired_velocity[1];
	velocitySqr += desired_velocity[2]*desired_velocity[2];

	float dfFactor = sqrt((h/g) + (velocitySqr/(4*g*g)));

	df[0] = desired_velocity[0] * dfFactor;
	df[1] = desired_velocity[1] * dfFactor;
	df[2] = desired_velocity[2] * dfFactor;

	prev_stepping_location[leg_id][0] = next_stepping_location[leg_id][0];
	prev_stepping_location[leg_id][1] = next_stepping_location[leg_id][1];
	prev_stepping_location[leg_id][2] = next_stepping_location[leg_id][2];

	next_stepping_location[leg_id][0] = df[0] + (current_velocity[0] - desired_velocity[0])*sqrt(h/g);
	next_stepping_location[leg_id][1] = df[1] + (current_velocity[1] - desired_velocity[1])*sqrt(h/g);
	next_stepping_location[leg_id][2] = df[2] + (current_velocity[2] - desired_velocity[2])*sqrt(h/g);
}