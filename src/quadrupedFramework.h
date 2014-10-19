#ifndef QUADRUPED_FRAMEWORK_H
#define QUADRUPED_FRAMEWORK_H

#include "ODEBodies.h"
#include "controller.h"
#include "QSMatrix.h"

class quadrupedFramework
{
private:
	//Private Objects
	ODEBodies * body_bag;
    controller * gait_controller;

    float root_position[3];

public:
	//Constructor
    quadrupedFramework();

    //Public Methods
    void takeStep();
    
    //Getters
    ODEBodies * getBodyBag();
    controller * getGaitController();
    float * getRootPosition();
};

#endif // QUADRUPED_FRAMEWORK_H
