#include "controller.h"

controller::controller(){
	gait_graph = new gaitGraph();
	foot_location = new footLocation();

	time = 0;
	T = 100;
	swingStart[0] = 0;
	swingEnd[0] = 50;
	swingStart[1] = 0;
	swingEnd[1] = 50;
	swingStart[2] = 50;
	swingEnd[2] = 100;
	swingStart[3] = 50;
	swingEnd[3] = 100;

	shoulderHeight = 47;		//in cm
    hipHeight = 45;

    current_velocity[0] = 200;		//in cm/s
    current_velocity[1] = 0;
    current_velocity[2] = 0;
    desired_velocity[0] = 400;
    desired_velocity[1] = 0;
    desired_velocity[2] = 0;
}

void controller::takeStep(){
	time = (time + 1);
	cout << "Time = " << time << endl;
	cout << "Leg 1 = " << gait_graph->isInSwing(time%T, swingStart[0], swingEnd[0]) << endl;
	cout << "Leg 2 = " << gait_graph->isInSwing(time%T, swingStart[1], swingEnd[1]) << endl;
	cout << "Leg 3 = " << gait_graph->isInSwing(time%T, swingStart[2], swingEnd[2]) << endl;
	cout << "Leg 4 = " << gait_graph->isInSwing(time%T, swingStart[3], swingEnd[3]) << endl;

	if(swingStart[0] == time%T){		
		foot_location->computePlacementLocation(shoulderHeight, current_velocity, desired_velocity);
	}
	if(swingStart[1] == time%T){		
		foot_location->computePlacementLocation(shoulderHeight, current_velocity, desired_velocity);
	}
	if(swingStart[2] == time%T){		
		foot_location->computePlacementLocation(hipHeight, current_velocity, desired_velocity);
	}
	if(swingStart[3] == time%T){		
		foot_location->computePlacementLocation(hipHeight, current_velocity, desired_velocity);
	}
}