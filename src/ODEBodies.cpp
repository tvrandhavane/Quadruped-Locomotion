#include "ODEBodies.h"

ODEBodies::ODEBodies(helper * global_helper){
	this->global_helper = global_helper;

    root_position[0] = -200.0;
    root_position[1] = 470.0 - 250.0;
    root_position[2] = 0.0;
}

void ODEBodies::init(){
	set_ball();
    set_back();
    set_nnh();
    set_tail();
	set_plane();
}

void ODEBodies::setRotationMatrixZAxis(dReal * R,float theta){
    R[0] = cos(theta);
    R[1] = sin(theta);
    R[2] = 0;
    R[3] = 0;

    R[4] = (-1)*sin(theta);
    R[5] = cos(theta);
    R[6] = 0;
    R[7] = 0;

    R[8] = 0;
    R[9] = 0;
    R[10] = 1;
    R[11] = 0;
}

void ODEBodies::set_ball(){
	ball_body = dBodyCreate(global_helper->getWorld());
    dMassSetZero(&ball_mass);
    dMassSetCylinderTotal(&ball_mass, 1, 2, 1.0, 10);
    dBodySetMass(ball_body, &ball_mass);
    dBodySetLinearVel(ball_body, -7.0, 0.0, 0.0);

    ball_geom = dCreateCylinder(global_helper->getSpace(), 1.0, 10.0);
    dGeomSetData(ball_geom, (void *)"ball");
    dGeomSetBody(ball_geom, ball_body);
    dGeomSetPosition(ball_geom, 0.0, 0.0, 0.0);
}

void ODEBodies::set_back(){
    //Link 1
    back_link_1_body = dBodyCreate(global_helper->getWorld());
    dMassSetZero(&back_link_1_mass);
    dMassSetCylinderTotal(&back_link_1_mass, 10, 2, 2.0, 80);
    dBodySetMass(back_link_1_body, &back_link_1_mass);
    dBodySetLinearVel(back_link_1_body, 0.0, 0.0, 0.0);

    back_link_1_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 80.0);
    dGeomSetData(back_link_1_geom, (void *)"back_link_1");
    dGeomSetBody(back_link_1_geom, back_link_1_body);
    dGeomSetPosition(back_link_1_geom, root_position[0], root_position[1], root_position[2]);

    //Link 2
    back_link_2_body = dBodyCreate(global_helper->getWorld());
    dMassSetZero(&back_link_2_mass);
    dMassSetCylinderTotal(&back_link_2_mass, 10, 2, 2.0, 80);
    dBodySetMass(back_link_2_body, &back_link_2_mass);
    dBodySetLinearVel(back_link_2_body, 0.0, 0.0, 0.0);

    back_link_2_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 80.0);
    dGeomSetData(back_link_2_geom, (void *)"back_link_2");
    dGeomSetBody(back_link_2_geom, back_link_2_body);
    dGeomSetPosition(back_link_2_geom, root_position[0] + 80.0, root_position[1], root_position[2]);

    //Link 3
    back_link_3_body = dBodyCreate(global_helper->getWorld());
    dMassSetZero(&back_link_3_mass);
    dMassSetCylinderTotal(&back_link_3_mass, 10, 2, 2.0, 80);
    dBodySetMass(back_link_3_body, &back_link_3_mass);
    dBodySetLinearVel(back_link_3_body, 0.0, 0.0, 0.0);

    back_link_3_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 80.0);
    dGeomSetData(back_link_3_geom, (void *)"back_link_3");
    dGeomSetBody(back_link_3_geom, back_link_3_body);
    dGeomSetPosition(back_link_3_geom, root_position[0] + 160.0, root_position[1], root_position[2]);

    //Link 4
    back_link_4_body = dBodyCreate(global_helper->getWorld());
    dReal * back_link_4_rotation_matrix;
    back_link_4_theta = (-5*M_PI)/180;
    back_link_4_rotation_matrix = new float(12);
    setRotationMatrixZAxis(back_link_4_rotation_matrix, back_link_4_theta);
    dMassSetZero(&back_link_4_mass);
    dMassSetCylinderTotal(&back_link_4_mass, 10, 2, 2.0, 80);
    dBodySetMass(back_link_4_body, &back_link_4_mass);
    dBodySetLinearVel(back_link_4_body, 0.0, 0.0, 0.0);
    dBodySetRotation(back_link_4_body, back_link_4_rotation_matrix);

    back_link_4_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 80.0);
    dGeomSetData(back_link_4_geom, (void *)"back_link_4");
    dGeomSetBody(back_link_4_geom, back_link_4_body);
    dGeomSetPosition(back_link_4_geom, root_position[0] + 240.0, root_position[1], root_position[2]);
    dGeomSetRotation(back_link_4_geom, back_link_4_rotation_matrix);

    //Link 5
    back_link_5_body = dBodyCreate(global_helper->getWorld());
    dReal * back_link_5_rotation_matrix;
    dMassSetZero(&back_link_5_mass);
    dMassSetCylinderTotal(&back_link_5_mass, 10, 2, 2.0, 80);
    dBodySetMass(back_link_5_body, &back_link_5_mass);
    dBodySetLinearVel(back_link_5_body, 0.0, 0.0, 0.0);

    back_link_5_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 80.0);
    dGeomSetData(back_link_5_geom, (void *)"back_link_5");
    dGeomSetBody(back_link_5_geom, back_link_5_body);
    dGeomSetPosition(back_link_5_geom, root_position[0] + 240.0 + 80*cos(back_link_4_theta), root_position[1] - 80*sin(back_link_4_theta), root_position[2]);

    //Link 6
    back_link_6_body = dBodyCreate(global_helper->getWorld());
    dReal * back_link_6_rotation_matrix;
    back_link_6_theta = asin((-80*sin(back_link_4_theta) + 20)/80);
    back_link_6_rotation_matrix = new float(12);
    setRotationMatrixZAxis(back_link_6_rotation_matrix, back_link_6_theta);
    dMassSetZero(&back_link_6_mass);
    dMassSetCylinderTotal(&back_link_6_mass, 10, 2, 2.0, 80);
    dBodySetMass(back_link_6_body, &back_link_6_mass);
    dBodySetLinearVel(back_link_6_body, 0.0, 0.0, 0.0);
    dBodySetRotation(back_link_6_body, back_link_6_rotation_matrix);

    back_link_6_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 80.0);
    dGeomSetData(back_link_6_geom, (void *)"back_link_6");
    dGeomSetBody(back_link_6_geom, back_link_6_body);
    dGeomSetPosition(back_link_6_geom, root_position[0] + 320.0 + 80*cos(back_link_4_theta), root_position[1] - 80*sin(back_link_4_theta), root_position[2]);
    dGeomSetRotation(back_link_6_geom, back_link_6_rotation_matrix);

    //Joints
    dJointID ball_joint_1_2 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(ball_joint_1_2, back_link_1_body, back_link_2_body);
    dJointSetBallAnchor (ball_joint_1_2, root_position[0] + 80.0, root_position[1], root_position[2]);

    dJointID ball_joint_2_3 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(ball_joint_2_3, back_link_2_body, back_link_3_body);
    dJointSetBallAnchor (ball_joint_2_3, root_position[0] + 160.0, root_position[1], root_position[2]);

    dJointID ball_joint_3_4 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(ball_joint_3_4, back_link_3_body, back_link_4_body);
    dJointSetBallAnchor (ball_joint_3_4, root_position[0] + 240.0, root_position[1], root_position[2]);

    dJointID ball_joint_4_5 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(ball_joint_4_5, back_link_4_body, back_link_5_body);
    dJointSetBallAnchor (ball_joint_4_5, root_position[0] + 240.0 + 80*cos(back_link_4_theta), root_position[1] - 80*sin(back_link_4_theta), root_position[2]);

    dJointID ball_joint_5_6 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(ball_joint_5_6, back_link_5_body, back_link_6_body);
    dJointSetBallAnchor (ball_joint_5_6, root_position[0] + 320.0 + 80*cos(back_link_4_theta), root_position[1] - 80*sin(back_link_4_theta), root_position[2]);
}

void ODEBodies::set_nnh(){
    //Link 1
    nnh_link_1_body = dBodyCreate(global_helper->getWorld());
    dReal * nnh_link_1_rotation_matrix;
    float nnh_link_1_theta = (-120*M_PI)/180;
    nnh_link_1_rotation_matrix = new float(12);
    setRotationMatrixZAxis(nnh_link_1_rotation_matrix, nnh_link_1_theta);
    dMassSetZero(&nnh_link_1_mass);
    dMassSetCylinderTotal(&nnh_link_1_mass, 10, 2, 2.0, 60);
    dBodySetMass(nnh_link_1_body, &nnh_link_1_mass);
    dBodySetLinearVel(nnh_link_1_body, 0.0, 0.0, 0.0);
    dBodySetRotation(nnh_link_1_body, nnh_link_1_rotation_matrix);

    nnh_link_1_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 60.0);
    dGeomSetData(nnh_link_1_geom, (void *)"nnh_link_1");
    dGeomSetBody(nnh_link_1_geom, nnh_link_1_body);
    dGeomSetPosition(nnh_link_1_geom, root_position[0], root_position[1], root_position[2]);
    dGeomSetRotation(nnh_link_1_geom, nnh_link_1_rotation_matrix);

    //Link 2
    nnh_link_2_body = dBodyCreate(global_helper->getWorld());
    dReal * nnh_link_2_rotation_matrix;
    float nnh_link_2_theta = (-90*M_PI)/180;
    nnh_link_2_rotation_matrix = new float(12);
    setRotationMatrixZAxis(nnh_link_2_rotation_matrix, nnh_link_2_theta);
    dMassSetZero(&nnh_link_2_mass);
    dMassSetCylinderTotal(&nnh_link_2_mass, 10, 2, 2.0, 60);
    dBodySetMass(nnh_link_2_body, &nnh_link_2_mass);
    dBodySetLinearVel(nnh_link_2_body, 0.0, 0.0, 0.0);
    dBodySetRotation(nnh_link_2_body, nnh_link_2_rotation_matrix);

    nnh_link_2_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 60.0);
    dGeomSetData(nnh_link_2_geom, (void *)"nnh_link_2");
    dGeomSetBody(nnh_link_2_geom, nnh_link_2_body);
    dGeomSetPosition(nnh_link_2_geom, root_position[0] - 30.0, root_position[1] + 60*sin((60*3.14)/180), root_position[2]);
    dGeomSetRotation(nnh_link_2_geom, nnh_link_2_rotation_matrix);

    //Link 3
    nnh_link_3_body = dBodyCreate(global_helper->getWorld());
    dReal * nnh_link_3_rotation_matrix;
    float nnh_link_3_theta = (-120*M_PI)/180;
    nnh_link_3_rotation_matrix = new float(12);
    setRotationMatrixZAxis(nnh_link_3_rotation_matrix, nnh_link_3_theta);
    dMassSetZero(&nnh_link_3_mass);
    dMassSetCylinderTotal(&nnh_link_3_mass, 10, 2, 2.0, 60);
    dBodySetMass(nnh_link_3_body, &nnh_link_3_mass);
    dBodySetLinearVel(nnh_link_3_body, 0.0, 0.0, 0.0);
    dBodySetRotation(nnh_link_3_body, nnh_link_3_rotation_matrix);

    nnh_link_3_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 60.0);
    dGeomSetData(nnh_link_3_geom, (void *)"nnh_link_3");
    dGeomSetBody(nnh_link_3_geom, nnh_link_3_body);
    dGeomSetPosition(nnh_link_3_geom, root_position[0] - 30.0, root_position[1] + 60 + 60*sin((60*3.14)/180), root_position[2]);
    dGeomSetRotation(nnh_link_3_geom, nnh_link_3_rotation_matrix);

    //Link 4;
    nnh_link_4_body = dBodyCreate(global_helper->getWorld());
    dReal * nnh_link_4_rotation_matrix;
    float nnh_link_4_theta = (-150*M_PI)/180;
    nnh_link_4_rotation_matrix = new float(12);
    setRotationMatrixZAxis(nnh_link_4_rotation_matrix, nnh_link_4_theta);
    dMassSetZero(&nnh_link_4_mass);
    dMassSetCylinderTotal(&nnh_link_4_mass, 10, 2, 2.0, 60);
    dBodySetMass(nnh_link_4_body, &nnh_link_4_mass);
    dBodySetLinearVel(nnh_link_4_body, 0.0, 0.0, 0.0);
    dBodySetRotation(nnh_link_4_body, nnh_link_4_rotation_matrix);

    nnh_link_4_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 60.0);
    dGeomSetData(nnh_link_4_geom, (void *)"nnh_link_4");
    dGeomSetBody(nnh_link_4_geom, nnh_link_4_body);
    dGeomSetPosition(nnh_link_4_geom, root_position[0] - 60.0, root_position[1] + 60 + 2*60*sin((60*3.14)/180), root_position[2]);
    dGeomSetRotation(nnh_link_4_geom, nnh_link_4_rotation_matrix);

    //Joints
    dJointID ball_joint_spine_1= dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(ball_joint_spine_1, back_link_1_body, nnh_link_1_body);
    dJointSetBallAnchor (ball_joint_spine_1, root_position[0], root_position[1], root_position[2]);
}

void ODEBodies::set_tail(){
    float start_location[3];

    start_location[0] = root_position[0] + 320.0 + 80*cos(back_link_4_theta) + 80*cos(back_link_6_theta);
    start_location[1] = root_position[1] - 80*sin(back_link_4_theta) - 80*sin(back_link_6_theta);
    start_location[2] = root_position[2];
    
    //Link 1
    tail_link_1_body = dBodyCreate(global_helper->getWorld());
    dReal * tail_link_1_rotation_matrix;
    float tail_link_1_theta = (30*M_PI)/180;
    tail_link_1_rotation_matrix = new float(12);
    setRotationMatrixZAxis(tail_link_1_rotation_matrix, tail_link_1_theta);
    dMassSetZero(&tail_link_1_mass);
    dMassSetCylinderTotal(&tail_link_1_mass, 10, 2, 2.0, 60);
    dBodySetMass(tail_link_1_body, &tail_link_1_mass);
    dBodySetLinearVel(tail_link_1_body, 0.0, 0.0, 0.0);
    dBodySetRotation(tail_link_1_body, tail_link_1_rotation_matrix);

    tail_link_1_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 60.0);
    dGeomSetData(tail_link_1_geom, (void *)"tail_link_1");
    dGeomSetBody(tail_link_1_geom, tail_link_1_body);
    dGeomSetPosition(tail_link_1_geom, start_location[0], start_location[1], start_location[2]);
    dGeomSetRotation(tail_link_1_geom, tail_link_1_rotation_matrix);

    //Link 2
    tail_link_2_body = dBodyCreate(global_helper->getWorld());
    dReal * tail_link_2_rotation_matrix;
    float tail_link_2_theta = (60*M_PI)/180;
    tail_link_2_rotation_matrix = new float(12);
    setRotationMatrixZAxis(tail_link_2_rotation_matrix, tail_link_2_theta);
    dMassSetZero(&tail_link_2_mass);
    dMassSetCylinderTotal(&tail_link_2_mass, 10, 2, 2.0, 60);
    dBodySetMass(tail_link_2_body, &tail_link_2_mass);
    dBodySetLinearVel(tail_link_2_body, 0.0, 0.0, 0.0);
    dBodySetRotation(tail_link_2_body, tail_link_2_rotation_matrix);

    tail_link_2_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 60.0);
    dGeomSetData(tail_link_2_geom, (void *)"tail_link_2");
    dGeomSetBody(tail_link_2_geom, tail_link_2_body);
    dGeomSetPosition(tail_link_2_geom, start_location[0] + 80*cos(tail_link_1_theta), start_location[1] - 80*sin(tail_link_1_theta), start_location[2]);
    dGeomSetRotation(tail_link_2_geom, tail_link_2_rotation_matrix);

    //Link 3
    tail_link_3_body = dBodyCreate(global_helper->getWorld());
    dReal * tail_link_3_rotation_matrix;
    float tail_link_3_theta = (90*M_PI)/180;
    tail_link_3_rotation_matrix = new float(12);
    setRotationMatrixZAxis(tail_link_3_rotation_matrix, tail_link_3_theta);
    dMassSetZero(&tail_link_3_mass);
    dMassSetCylinderTotal(&tail_link_3_mass, 10, 2, 2.0, 60);
    dBodySetMass(tail_link_3_body, &tail_link_3_mass);
    dBodySetLinearVel(tail_link_3_body, 0.0, 0.0, 0.0);
    dBodySetRotation(tail_link_3_body, tail_link_3_rotation_matrix);

    tail_link_3_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 60.0);
    dGeomSetData(tail_link_3_geom, (void *)"tail_link_3");
    dGeomSetBody(tail_link_3_geom, tail_link_3_body);
    dGeomSetPosition(tail_link_3_geom, start_location[0] + 80*cos(tail_link_1_theta) + 80*cos(tail_link_2_theta), start_location[1] - 80*sin(tail_link_1_theta) - 80*sin(tail_link_2_theta), start_location[2]);
    dGeomSetRotation(tail_link_3_geom, tail_link_3_rotation_matrix);

    //Link 4
    tail_link_4_body = dBodyCreate(global_helper->getWorld());
    dReal * tail_link_4_rotation_matrix;
    float tail_link_4_theta = (60*M_PI)/180;
    tail_link_4_rotation_matrix = new float(12);
    setRotationMatrixZAxis(tail_link_4_rotation_matrix, tail_link_4_theta);
    dMassSetZero(&tail_link_4_mass);
    dMassSetCylinderTotal(&tail_link_4_mass, 10, 2, 2.0, 60);
    dBodySetMass(tail_link_4_body, &tail_link_4_mass);
    dBodySetLinearVel(tail_link_4_body, 0.0, 0.0, 0.0);
    dBodySetRotation(tail_link_4_body, tail_link_4_rotation_matrix);

    tail_link_4_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 60.0);
    dGeomSetData(tail_link_4_geom, (void *)"tail_link_4");
    dGeomSetBody(tail_link_4_geom, tail_link_4_body);
    dGeomSetPosition(tail_link_4_geom, start_location[0] + 80*cos(tail_link_1_theta) + 80*cos(tail_link_2_theta), start_location[1] - 80*sin(tail_link_1_theta) - 80*sin(tail_link_2_theta) - 80, start_location[2]);
    dGeomSetRotation(tail_link_4_geom, tail_link_4_rotation_matrix);
}

void ODEBodies::set_plane(){
	plane_geom = dCreatePlane(global_helper->getSpace(), 0.0, 1.0, 0.0, -250.0);	
}

helper * ODEBodies::getGlobalHelper(){
    return global_helper;
}

dBodyID ODEBodies::getBallBody(){
	return ball_body;
}

dBodyID ODEBodies::getBackLink1Body(){
    return back_link_1_body;
}

dBodyID ODEBodies::getBackLink2Body(){
    return back_link_2_body;
}

dBodyID ODEBodies::getBackLink3Body(){
    return back_link_3_body;
}

dBodyID ODEBodies::getBackLink4Body(){
    return back_link_4_body;
}

dBodyID ODEBodies::getBackLink5Body(){
    return back_link_5_body;
}

dBodyID ODEBodies::getBackLink6Body(){
    return back_link_6_body;
}

dBodyID ODEBodies::getNnhLink1Body(){
    return nnh_link_1_body;
}

dBodyID ODEBodies::getNnhLink2Body(){
    return nnh_link_2_body;
}

dBodyID ODEBodies::getNnhLink3Body(){
    return nnh_link_3_body;
}

dBodyID ODEBodies::getNnhLink4Body(){
    return nnh_link_4_body;
}

dBodyID ODEBodies::getTailLink1Body(){
    return tail_link_1_body;
}

dBodyID ODEBodies::getTailLink2Body(){
    return tail_link_2_body;
}

dBodyID ODEBodies::getTailLink3Body(){
    return tail_link_3_body;
}

dBodyID ODEBodies::getTailLink4Body(){
    return tail_link_4_body;
}

dGeomID ODEBodies::getPlaneGeom(){
	return plane_geom;
}




