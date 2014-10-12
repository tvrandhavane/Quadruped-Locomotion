#include "quadrupedFramework.h"

quadrupedFramework::quadrupedFramework(){
    //Initialize controller
    gait_controller = new controller();

    root_position[0] = -200.0;
    root_position[1] = 470.0 - 250.0;
    root_position[2] = 200.0;

    //Initialize body bag
    helper * global_helper = new helper();
    global_helper->init();  //initiaize world, space and contact group   
    body_bag = new ODEBodies(global_helper, root_position);
    body_bag->init();
}

void quadrupedFramework::takeStep(){
    vector<float> lengths = body_bag->getLengths(0);
    vector<float> angles = body_bag->getAngles(0);
    vector<float> endEffector(3);
    const dReal *endEffectorPos = dBodyGetPosition(body_bag->getFrontLeftFootLink4Body());
    endEffector[0] = endEffectorPos[0]-body_bag->getFrontFootLink4Length()/2;
    endEffector[1] = endEffectorPos[1];
    endEffector[2] = endEffectorPos[2];
    gait_controller->applyIK(lengths, angles, endEffector);
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
