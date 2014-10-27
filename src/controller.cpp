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
    	//Alignment from my co-ordinate system to ODE system
    	Eigen::MatrixXf alignment = Eigen::MatrixXf::Identity(4, 4);
		alignment(1, 1) = 0;
		alignment(1, 2) = 1;
		alignment(2, 1) = -1;
		alignment(2, 2) = 0;

    	const dReal *front_left_foot_link_4_location = dBodyGetPosition(body_bag->getFootLinkBody(i,3));

    	//translation
		Eigen::MatrixXf mt = Eigen::MatrixXf::Identity(4, 4);
		mt(1, 3) = body_bag->getFootLinkLength(i, 3)/2;

    	//rotation
		Eigen::MatrixXf mr = Eigen::MatrixXf::Identity(4, 4);
		const dReal *rotation_matrix_ode = dBodyGetRotation(body_bag->getFootLinkBody(i, 3));
		for(int k = 0; k < 3; k++){
			for(int j = 0; j < 4; j++){
				mr(k, j) = rotation_matrix_ode[k*4+j];
			}
		}
		mr(3, 0) = 0;
		mr(3, 1) = 0;
		mr(3, 2) = 0;
		mr(3, 3) = 1;

		Eigen::MatrixXf origin = Eigen::MatrixXf::Random(4, 1);
		origin(0, 0) = 0;
		origin(1, 0) = 0;
		origin(2, 0) = 0;
		origin(3, 0) = 1;

		Eigen::MatrixXf addition = alignment.inverse()*mr.inverse()*mt*origin;
    	prev_stepping_location[i][0] = front_left_foot_link_4_location[0] + addition(0, 0);
    	prev_stepping_location[i][1] = front_left_foot_link_4_location[1] + addition(1, 0);
    	prev_stepping_location[i][2] = front_left_foot_link_4_location[2] + addition(2, 0);
    	next_stepping_location[i][0] = prev_stepping_location[i][0];
    	next_stepping_location[i][1] = prev_stepping_location[i][1];
    	next_stepping_location[i][2] = prev_stepping_location[i][2];
    	current_foot_location[i][0] = prev_stepping_location[i][0];
    	current_foot_location[i][1] = prev_stepping_location[i][1];
    	current_foot_location[i][2] = prev_stepping_location[i][2];
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

    gravityCompensation();
}

void controller::legController(int leg_id, int phase){
	if(swingStart[leg_id] == phase){		
		computePlacementLocation(leg_id, lfHeight[0]);
		setFootLocation(leg_id, phase);
	}
	else if(isInSwing(leg_id)){
		//Get target position
		setFootLocation(leg_id, phase);
		vector<float> targetPosition = getTargetPosition(leg_id);
		//Get link lengths
		vector<float> lengths(2);
		for(int i = 0; i < 2; i++){
			lengths[i] = body_bag->getFootLinkLength(leg_id, i);
		}
		//Set axis perpendicular to the kinematic chain plane
		Eigen::MatrixXf axis = Eigen::MatrixXf::Random(4, 1);
		axis(0, 0) = 0;
		axis(1, 0) = 0;
		if(leg_id % 2 == 0){
    		axis(2, 0) = 1;
    	}
    	else{
    		axis(2, 0) = -1;
    	}   		
    	axis(3, 0) = 1;
    	
    	//Get transformation matrices
    	vector<Eigen::MatrixXf> transformationMatrices(2);
    	Eigen::MatrixXf alignment = Eigen::MatrixXf::Identity(4, 4);
		alignment(1, 1) = 0;
		alignment(1, 2) = 1;
		alignment(2, 1) = -1;
		alignment(2, 2) = 0;

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

		Eigen::MatrixXf mr2 = Eigen::MatrixXf::Identity(4, 4);
		const dReal *rotation_matrix_ode2 = dBodyGetRotation(body_bag->getFootLinkBody(leg_id, 3));
		for(int i = 0; i < 3; i++){
			for(int j = 0; j < 4; j++){
				mr2(i, j) = rotation_matrix_ode2[i*4+j];
			}
		}
		mr2(3, 0) = 0;
		mr2(3, 1) = 0;
		mr2(3, 2) = 0;
		mr2(3, 3) = 1;

		transformationMatrices[0] = mr*alignment.inverse();
		transformationMatrices[1] = mr2*alignment.inverse();
		//Get translation matrix
		const dReal *center_location = dBodyGetPosition(body_bag->getFootLinkBody(leg_id,0)); //get location of the center of the link
		Eigen::MatrixXf mt = Eigen::MatrixXf::Identity(4, 4); //translate to the start of the link
		mt(1, 3) = -body_bag->getFootLinkLength(leg_id, 0)/2;
		mr = Eigen::MatrixXf::Identity(4, 4); //get orientation
		rotation_matrix_ode = dBodyGetRotation(body_bag->getFootLinkBody(leg_id, 0));
		for(int k = 0; k < 3; k++){
			for(int j = 0; j < 4; j++){
				mr(k, j) = rotation_matrix_ode[k*4+j];
			}
		}
		mr(3, 0) = 0;
		mr(3, 1) = 0;
		mr(3, 2) = 0;
		mr(3, 3) = 1;
		Eigen::MatrixXf origin = Eigen::MatrixXf::Random(4, 1);
		origin(0, 0) = 0;
		origin(1, 0) = 0;
		origin(2, 0) = 0;
		origin(3, 0) = 1;
		Eigen::MatrixXf addition = alignment.inverse()*mr.inverse()*mt*origin; //part to add to the center location
		Eigen::MatrixXf translationMatrix = Eigen::MatrixXf::Identity(4, 4);
	    translationMatrix(0, 3) = center_location[0] + addition(0, 0);
	    translationMatrix(1, 3) = center_location[1] + addition(1, 0);
	    translationMatrix(2, 3) = center_location[2] + addition(2, 0);

	    vector<float> angles = body_bag->getAngles(0);
	    applyIK(lengths, transformationMatrices, angles, targetPosition, translationMatrix, axis);
		//Apply IK and get change in angles
		//Use PD controllers to get torque
	}
	else{
		stanceLegTreatment(leg_id);
	}
}

vector<float> controller::getTargetPosition(int leg_id){
	//Last link
	//translation
	Eigen::MatrixXf mt = Eigen::MatrixXf::Identity(4, 4);
	mt(2, 3) = body_bag->getFootLinkLength(leg_id, 3);	
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

	Eigen::MatrixXf point = Eigen::MatrixXf::Random(4, 1);
	point(0, 0) = current_foot_location[leg_id][0];
	point(1, 0) = current_foot_location[leg_id][1];
	point(2, 0) = current_foot_location[leg_id][2];
	point(3, 0) = 1;
	Eigen::MatrixXf transformedPoint = mr*mt*mr.inverse()*point;

	//Second last link
	//translation
	mt = Eigen::MatrixXf::Identity(4, 4);
	mt(2, 3) = body_bag->getFootLinkLength(leg_id, 2);	
	//rotation
	mr = Eigen::MatrixXf::Identity(4, 4);
	rotation_matrix_ode = dBodyGetRotation(body_bag->getFootLinkBody(leg_id, 2));
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 4; j++){
			mr(i, j) = rotation_matrix_ode[i*4+j];
		}
	}
	mr(3, 0) = 0;
	mr(3, 1) = 0;
	mr(3, 2) = 0;
	mr(3, 3) = 1;

	Eigen::MatrixXf endEffectorM = mr*mt*mr.inverse()*transformedPoint;

	vector<float> endEffector(3);
	endEffector[0] = endEffectorM(0,0);
	endEffector[1] = endEffectorM(1,0);
	endEffector[2] = endEffectorM(2,0);
	return endEffector;
}

void controller::applyIK(vector<float> lengths, vector<Eigen::MatrixXf> transformationMatrices, vector<float> angles, vector<float> endEffector, Eigen::MatrixXf translationMatrix, Eigen::MatrixXf axis){
	inverseKinematics * IKSolver = new inverseKinematics(lengths, transformationMatrices, angles, endEffector, translationMatrix, axis);
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

	float temp[3];

	temp[0] = next_stepping_location[leg_id][0];
	temp[1] = next_stepping_location[leg_id][1];
	temp[2] = next_stepping_location[leg_id][2];

	next_stepping_location[leg_id][0] = prev_stepping_location[leg_id][0] + df[0] + (current_velocity[0] - desired_velocity[0])*sqrt(h/g);
	next_stepping_location[leg_id][1] = prev_stepping_location[leg_id][1] + df[1] + (current_velocity[1] - desired_velocity[1])*sqrt(h/g);
	next_stepping_location[leg_id][2] = prev_stepping_location[leg_id][2] + df[2] + (current_velocity[2] - desired_velocity[2])*sqrt(h/g);

	prev_stepping_location[leg_id][0] = temp[0];
	prev_stepping_location[leg_id][1] = temp[1];
	prev_stepping_location[leg_id][2] = temp[2];
}