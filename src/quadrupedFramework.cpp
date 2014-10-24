#include "quadrupedFramework.h"

quadrupedFramework::quadrupedFramework(){
    root_position[0] = -200.0;
    root_position[1] = 470.0 - 250.0;
    root_position[2] = 200.0;

    //Initialize body bag
    helper * global_helper = new helper();
    global_helper->init();  //initiaize world, space and contact group   
    body_bag = new ODEBodies(global_helper, root_position);
    body_bag->init();

    //Initialize controller
    gait_controller = new controller(body_bag, root_position);
}

void quadrupedFramework::takeStep(){
    //Controller step
    gait_controller->takeStep();
}

ODEBodies * quadrupedFramework::getBodyBag(){
    return body_bag;
}

controller * quadrupedFramework::getGaitController(){
    return gait_controller;
}

float * quadrupedFramework::getRootPosition(){
    return root_position;
}
