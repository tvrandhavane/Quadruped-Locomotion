#include "controller.h"

controller::controller(ODEBodies * body_bag){
	time = 0;
	T = 100;
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

void controller::applyIK(vector<float> lengths, vector<float> angles, vector<float> endEffector, QSMatrix<float> transformationMatrix, QSMatrix<float> axis){
	inverseKinematics * IKSolver = new inverseKinematics(lengths, angles, endEffector, transformationMatrix, axis);
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
    dBodyAddForce(body_bag->getFrontLeftFootLink1Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getFrontLeftFootLink2Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getFrontLeftFootLink3Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getFrontLeftFootLink4Body(), 0.0, 98.1, 0.0);
    //Front right foot    
    dBodyAddForce(body_bag->getFrontRightFootLink1Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getFrontRightFootLink2Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getFrontRightFootLink3Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getFrontRightFootLink4Body(), 0.0, 98.1, 0.0);
    //Back left foot
    dBodyAddForce(body_bag->getBackLeftFootLink1Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getBackLeftFootLink2Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getBackLeftFootLink3Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getBackLeftFootLink4Body(), 0.0, 98.1, 0.0);
    //Back right foot    
    dBodyAddForce(body_bag->getBackRightFootLink1Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getBackRightFootLink2Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getBackRightFootLink3Body(), 0.0, 98.1, 0.0);
    dBodyAddForce(body_bag->getBackRightFootLink4Body(), 0.0, 98.1, 0.0);
}

void controller::takeStep(){
	time = (time + 1);
	int phase = time%T;

	cout << "Time = " << time << endl;

	//for each leg
	for(int i = 0; i < 4; i++){
		if(swingStart[i] == phase){		
			computePlacementLocation(i, lfHeight[0]);
			setFootLocation(i, phase);
		}
		else if(isInSwing(i)){
			setFootLocation(i, phase);
		}
		else{
			stanceLegTreatment(i);
		}
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