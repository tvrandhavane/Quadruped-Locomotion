#include "controller.h"

controller::controller(ODEBodies * body_bag, float * root_position){
	time = 0;
	T = 100;

	this->root_position = root_position;

	swingStart[0] = 1;
	swingEnd[0] = 50;
	swingStart[1] = 51;
	swingEnd[1] = 100;
	swingStart[2] = 1;
	swingEnd[2] = 50;
	swingStart[3] = 51;
	swingEnd[3] = 100;

	shoulderHeight = 470;		//in 0.1cm
    hipHeight = 450;

    current_velocity[0] = 0;		//in 0.1cm/s
    current_velocity[1] = 0;
    current_velocity[2] = 0;
    desired_velocity[0] = -1000;
    desired_velocity[1] = 0;
    desired_velocity[2] = 0;

    lfHeight[0] = shoulderHeight;
    lfHeight[1] = shoulderHeight;
    lfHeight[2] = hipHeight;
    lfHeight[3] = hipHeight;

    for(int i = 0; i < 4; i++){
    	swingFlag[i] = false;
    	for(int j = 0; j < 4; j++){
    		foot_link_pd_error[i][j] = 0;
    	}
    	foot_link_gain_kp[i][0] = 1000;
    	foot_link_gain_kd[i][0] = 1000;
    	foot_link_gain_kp[i][1] = 1000;
    	foot_link_gain_kd[i][1] = 1000;
    	foot_link_gain_kp[i][2] = 100;
    	foot_link_gain_kd[i][2] = 100;
    	foot_link_gain_kp[i][3] = -700;
    	foot_link_gain_kd[i][3] = 10;
    	for(int j = 0; j < 3; j++){
    		swing_torque[i][j] = 0;
    	}
    }

    lf_velocity_force_kv[0] = 0.01;
    lf_velocity_force_kv[1] = 0.01;

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
	virtualForces();
    gravityCompensation();
    cout << endl;
}

void controller::legController(int leg_id, int phase){
	cout << endl << "Leg Controller = " << leg_id << endl;
	if(swingStart[leg_id] == phase){
		cout << "Starting swing phase"<< endl;			
		computePlacementLocation(leg_id, lfHeight[0]);
		setFootLocation(leg_id, phase);
	}
	else if(isInSwing(leg_id)){
		cout << "Swing phase"<< endl;
		swingFlag[leg_id] = true;

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
	    Eigen::MatrixXf jointAngleChange = applyIK(lengths, transformationMatrices, angles, targetPosition, translationMatrix, axis);

		//Use PD controllers to get torque
		//link 1
	    float torque = foot_link_gain_kp[leg_id][0]*jointAngleChange(0,0) + foot_link_gain_kd[leg_id][0]*(jointAngleChange(0,0) - foot_link_pd_error[leg_id][0]);
		dBodyAddTorque(body_bag->getFootLinkBody(leg_id,0), axis(0, 0)*torque, axis(1, 0)*torque, axis(2, 0)*torque);
		for(int i = 0; i < 3; i++){
			swing_torque[leg_id][i] = axis(i, 0)*torque;
		}		
		foot_link_pd_error[leg_id][0] = jointAngleChange(0,0);

		//link 2
		torque = foot_link_gain_kp[leg_id][1]*jointAngleChange(1,0) + foot_link_gain_kd[leg_id][1]*(jointAngleChange(1,0) - foot_link_pd_error[leg_id][1]);
		dBodyAddTorque(body_bag->getFootLinkBody(leg_id,1), axis(0, 0)*torque, axis(1, 0)*torque, axis(2, 0)*torque);
		foot_link_pd_error[leg_id][1] = jointAngleChange(1,0);

		//link 3
		rotation_matrix_ode = dBodyGetRotation(body_bag->getFootLinkBody(leg_id, 2));
		for(int k = 0; k < 3; k++){
			for(int j = 0; j < 4; j++){
				mr(k, j) = rotation_matrix_ode[k*4+j];
			}
		}
		mr(3, 0) = 0;
		mr(3, 1) = 0;
		mr(3, 2) = 0;
		mr(3, 3) = 1;
		float swing_phase = computeSwingPhase(leg_id, phase);
		float error = ((30*M_PI)/180)*(1 - swing_phase) - ((60*M_PI)/180)*swing_phase - acos(mr(0, 0));
		torque = foot_link_gain_kp[leg_id][2]*error + foot_link_gain_kd[leg_id][2]*(error - foot_link_pd_error[leg_id][2]);
		dBodyAddTorque(body_bag->getFootLinkBody(leg_id,2), axis(0, 0)*torque, axis(1, 0)*torque, axis(2, 0)*torque);
		foot_link_pd_error[leg_id][2] = error;

		//link 4
		rotation_matrix_ode = dBodyGetRotation(body_bag->getFootLinkBody(leg_id, 3));
		for(int k = 0; k < 3; k++){
			for(int j = 0; j < 4; j++){
				mr(k, j) = rotation_matrix_ode[k*4+j];
			}
		}
		mr(3, 0) = 0;
		mr(3, 1) = 0;
		mr(3, 2) = 0;
		mr(3, 3) = 1;
		swing_phase = computeSwingPhase(leg_id, phase);
		error = ((90*M_PI)/180)*(1 - swing_phase) - ((30*M_PI)/180)*swing_phase - acos(mr(0, 0));
		torque = foot_link_gain_kp[leg_id][3]*error + foot_link_gain_kd[leg_id][3]*(error - foot_link_pd_error[leg_id][3]);
		dBodyAddTorque(body_bag->getFootLinkBody(leg_id,3), axis(0, 0)*torque, axis(1, 0)*torque, axis(2, 0)*torque);
		foot_link_pd_error[leg_id][3] = error;


		//Apply gravity compensation
		float force = 9.810*body_bag->getDensity()*(body_bag->getFootLinkLength(leg_id, 0) + body_bag->getFootLinkLength(leg_id, 1) + body_bag->getFootLinkLength(leg_id, 2) + body_bag->getFootLinkLength(leg_id, 3));

		dBodyAddForce(body_bag->getFootLinkBody(leg_id, 2), 0.0, 9.810*body_bag->getDensity()*body_bag->getFootLinkLength(leg_id, 2), 0.0);

		if(leg_id < 2){
    		dBodyAddForce(body_bag->getBackLink1Body(), 0.0, force, 0.0);
    	}
    	else{
    		dBodyAddForce(body_bag->getBackLink6Body(), 0.0, force, 0.0);
    	}
	}
	else{
		cout << "Stance phase"<< endl;
		swingFlag[leg_id] = false;
		/*for(int i = 0; i < 3; i++){
			swing_torque[leg_id][i] = 0;
		}*/
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

Eigen::MatrixXf controller::applyIK(vector<float> lengths, vector<Eigen::MatrixXf> transformationMatrices, vector<float> angles, vector<float> endEffector, Eigen::MatrixXf translationMatrix, Eigen::MatrixXf axis){
	inverseKinematics * IKSolver = new inverseKinematics(lengths, transformationMatrices, angles, endEffector, translationMatrix, axis);
	Eigen::MatrixXf jointAngleChange = IKSolver->getJointAngleChange();
	delete IKSolver;
	return jointAngleChange;
}

void controller::gravityCompensation(){
	dBodyAddForce(body_bag->getBackLink1Body(), 0.0, 9.810*body_bag->getDensity()*body_bag->getBackLink1Length(), 0.0);
    dBodyAddForce(body_bag->getBackLink2Body(), 0.0, 9.810*body_bag->getDensity()*body_bag->getBackLink2Length(), 0.0);
    dBodyAddForce(body_bag->getBackLink3Body(), 0.0, 9.810*body_bag->getDensity()*body_bag->getBackLink3Length(), 0.0);
    dBodyAddForce(body_bag->getBackLink4Body(), 0.0, 9.810*body_bag->getDensity()*body_bag->getBackLink4Length(), 0.0);
    dBodyAddForce(body_bag->getBackLink5Body(), 0.0, 9.810*body_bag->getDensity()*body_bag->getBackLink5Length(), 0.0);
    dBodyAddForce(body_bag->getBackLink6Body(), 0.0, 9.810*body_bag->getDensity()*body_bag->getBackLink6Length(), 0.0);
   /*//Neck and head
    float force = 9.810*body_bag->getDensity()*(body_bag->getNnhLinkLength(0) + body_bag->getNnhLinkLength(1) + body_bag->getNnhLinkLength(2) + body_bag->getNnhLinkLength(3));
	dBodyAddForce(body_bag->getBackLink1Body(), 0.0, force, 0.0);

		
    dBodyAddForce(body_bag->getNnhLink1Body(), 0.0, 9.810*body_bag->getDensity()*body_bag->getNnhLinkLength(0), 0.0);
    dBodyAddForce(body_bag->getNnhLink2Body(), 0.0, 9.810*body_bag->getDensity()*body_bag->getNnhLinkLength(1), 0.0);
    dBodyAddForce(body_bag->getNnhLink3Body(), 0.0, 9.810*body_bag->getDensity()*body_bag->getNnhLinkLength(2), 0.0);
    dBodyAddForce(body_bag->getNnhLink4Body(), 0.0, 9.810*body_bag->getDensity()*body_bag->getNnhLinkLength(3), 0.0);
    //Tail
    dBodyAddForce(body_bag->getTailLink1Body(), 0.0, 9.810*body_bag->getDensity()*body_bag->getTailLinkLength(0), 0.0);
    dBodyAddForce(body_bag->getTailLink2Body(), 0.0, 9.810*body_bag->getDensity()*body_bag->getTailLinkLength(1), 0.0);
    dBodyAddForce(body_bag->getTailLink3Body(), 0.0, 9.810*body_bag->getDensity()*body_bag->getTailLinkLength(2), 0.0);
    dBodyAddForce(body_bag->getTailLink4Body(), 0.0, 9.810*body_bag->getDensity()*body_bag->getTailLinkLength(3), 0.0);*/
    /*//Front left foot
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
    dBodyAddForce(body_bag->getFootLinkBody(3,3), 0.0, 98.1, 0.0);*/
}

void controller::virtualForces(){
	//For front leg frame
	if(swingFlag[0] == false || swingFlag[1] == false){
		float force = lf_velocity_force_kv[0]*(desired_velocity[0] - current_velocity[0]);
		dBodyAddForce(body_bag->getBackLink1Body(), force, 0.0, 0.0);
	}
	if(swingFlag[2] == false || swingFlag[3] == false){
		float force = lf_velocity_force_kv[1]*(desired_velocity[0] - current_velocity[0]);
		dBodyAddForce(body_bag->getBackLink6Body(), force, 0.0, 0.0);
	}
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
	int other_leg_id;
	if(leg_id == 0){
		other_leg_id = 1;
	}
	else if(leg_id == 1){
		other_leg_id = 0;
	}
	else if(leg_id == 2){
		other_leg_id = 3;
	}
	else if(leg_id == 3){
		other_leg_id = 2;
	}
	if(swingFlag[other_leg_id]){
		if(leg_id < 2){
			const dReal * torque =  dBodyGetTorque(body_bag->getFootLinkBody(other_leg_id,0));
			dBodyAddTorque(body_bag->getBackLink1Body(), -torque[0], -torque[1], -torque[2]);
		}
		else{
			const dReal * torque =  dBodyGetTorque(body_bag->getFootLinkBody(other_leg_id,0));
			dBodyAddTorque(body_bag->getBackLink6Body(), -torque[0], -torque[1], -torque[2]);
		}		
	}
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