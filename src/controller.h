#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
#include <cmath>
#include "footLocation.h"

using namespace std;

class controller
{
private:
	//Private Objects
    int time;
    int T;
    int swingStart[4];
    int swingEnd[4];

    int shoulderHeight;
    int hipHeight;

    float current_velocity[3];
    float desired_velocity[3];

    footLocation * foot_location;

	//Private Methods
    bool isInSwing(int leg_id);

public:
    //Constructor
    controller();
    
    //Public Methods
    void takeStep();

};

#endif // CONTROLLER_H
