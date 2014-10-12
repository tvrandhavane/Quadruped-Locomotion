#include "controller.h"

controller::controller(){
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

	root_position[0] = -200.0;
    root_position[1] = 470.0 - 250.0;
    root_position[2] = 200.0;

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

    //IKSolver = new inverseKinematics();
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

float * controller::getRootPosition(){
	return root_position;
}