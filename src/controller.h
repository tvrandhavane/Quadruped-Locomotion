#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
#include <cmath>
#include "gaitGraph.h"

using namespace std;

class controller
{
private:
	//Private Objects
    int time;
    int T;
    int swingStart[4];
    int swingEnd[4];
    gaitGraph * gait_graph;

	//Private Methods
    
public:
    //Constructor
    controller();
    
    //Public Methods
    void takeStep();

};

#endif // CONTROLLER_H
