#include "controller.h"

controller::controller(){
	gait_graph = new gaitGraph();
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
}

void controller::takeStep(){
	time = (time + 1);
	cout << "Time = " << time << endl;
	cout << "Leg 1 = " << gait_graph->isInSwing(time%T, swingStart[0], swingEnd[0]) << endl;
	cout << "Leg 2 = " << gait_graph->isInSwing(time%T, swingStart[1], swingEnd[1]) << endl;
	cout << "Leg 3 = " << gait_graph->isInSwing(time%T, swingStart[2], swingEnd[2]) << endl;
	cout << "Leg 4 = " << gait_graph->isInSwing(time%T, swingStart[3], swingEnd[3]) << endl;
}