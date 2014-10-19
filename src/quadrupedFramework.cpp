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
    gait_controller = new controller(body_bag);
}

void quadrupedFramework::takeStep(){
    vector<float> lengths = body_bag->getLengths(0);
    vector<float> angles = body_bag->getAngles(0);
    vector<float> endEffector(3);
    const dReal *endEffectorPos = dBodyGetPosition(body_bag->getFrontLeftFootLink4Body());
    endEffector[0] = endEffectorPos[0]-body_bag->getFrontFootLink4Length()/2;
    endEffector[1] = endEffectorPos[1];
    endEffector[2] = endEffectorPos[2];
    QSMatrix<float> transformationMatrix(4, 4, 0.0);
    transformationMatrix(0, 0) = 1;
    transformationMatrix(1, 1) = 1;
    transformationMatrix(2, 2) = 1;
    transformationMatrix(3, 3) = 1;
    transformationMatrix(2, 3) = 120;
    QSMatrix<float> axis(4, 1, 0.0);
    axis(2, 0) = 1;
    axis(3, 0) = 1;
    gait_controller->applyIK(lengths, angles, endEffector, transformationMatrix, axis);
    gait_controller->gravityCompensation();
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
