#include "ODEBodies.h"

ODEBodies::ODEBodies(helper * global_helper){
	this->global_helper = global_helper;
}

void ODEBodies::init(){
	set_ball();
    set_back();
	set_plane();
}

void ODEBodies::set_ball(){
	ball_body = dBodyCreate(global_helper->getWorld());
    dMassSetZero(&ball_mass);
    dMassSetCylinderTotal(&ball_mass, 1, 2, 1.0, 10);
    //dMassSetSphereTotal(&ball_mass, 0.1, 0.2);
    dBodySetMass(ball_body, &ball_mass);
    dBodySetLinearVel(ball_body, -7.0, 0.0, 0.0);

    ball_geom = dCreateCylinder(global_helper->getSpace(), 1.0, 10.0);
    dGeomSetData(ball_geom, (void *)"ball");
    dGeomSetBody(ball_geom, ball_body);
    dGeomSetPosition(ball_geom, 0.0, 0.0, 0.0);
}

void ODEBodies::set_back(){
    float root_position[3];

    root_position[0] = 0.0;
    root_position[1] = 470.0 - 250.0;
    root_position[2] = 0.0;
    
    dQuaternion q;
    q[0] = 3.14;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 1.0;

    /*back_link_1_body = dBodyCreate(global_helper->getWorld());
    dMassSetZero(&back_link_1_mass);
    dMassSetCylinderTotal(&back_link_1_mass, 1, 2, 2.0, 80);
    dBodySetMass(back_link_1_body, &back_link_1_mass);
    dBodySetLinearVel(back_link_1_body, 0.0, 0.0, 0.0);

    back_link_1_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 80.0);
    dGeomSetData(back_link_1_geom, (void *)"back_link_1");
    dGeomSetBody(back_link_1_geom, back_link_1_body);
    dGeomSetPosition(back_link_1_geom, root_position[0], root_position[1], root_position[2]);
    dBodySetQuaternion(back_link_1_body, q);*/

    back_link_1_body = dBodyCreate(global_helper->getWorld());
    dMassSetZero(&back_link_1_mass);
    dMassSetCylinderTotal(&back_link_1_mass, 10, 2, 2.0, 80);
    dBodySetMass(back_link_1_body, &back_link_1_mass);
    dBodySetLinearVel(back_link_1_body, 0.0, 0.0, 0.0);

    back_link_1_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 80.0);
    dGeomSetData(back_link_1_geom, (void *)"back_link_1");
    dGeomSetBody(back_link_1_geom, back_link_1_body);
    dGeomSetPosition(back_link_1_geom, root_position[0], root_position[1], root_position[2]);

    back_link_2_body = dBodyCreate(global_helper->getWorld());
    dMassSetZero(&back_link_2_mass);
    dMassSetCylinderTotal(&back_link_2_mass, 10, 2, 2.0, 80);
    dBodySetMass(back_link_2_body, &back_link_2_mass);
    dBodySetLinearVel(back_link_2_body, 0.0, 0.0, 0.0);

    back_link_2_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 80.0);
    dGeomSetData(back_link_2_geom, (void *)"back_link_2");
    dGeomSetBody(back_link_2_geom, back_link_2_body);
    dGeomSetPosition(back_link_2_geom, root_position[0] + 80.0, root_position[1], root_position[2]);

    back_link_3_body = dBodyCreate(global_helper->getWorld());
    dMassSetZero(&back_link_3_mass);
    dMassSetCylinderTotal(&back_link_3_mass, 10, 2, 2.0, 80);
    dBodySetMass(back_link_3_body, &back_link_3_mass);
    dBodySetLinearVel(back_link_3_body, 0.0, 0.0, 0.0);

    back_link_3_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 80.0);
    dGeomSetData(back_link_3_geom, (void *)"back_link_3");
    dGeomSetBody(back_link_3_geom, back_link_3_body);
    dGeomSetPosition(back_link_3_geom, root_position[0] + 160.0, root_position[1], root_position[2]);

    back_link_4_body = dBodyCreate(global_helper->getWorld());
    dMassSetZero(&back_link_4_mass);
    dReal * R = new float(12);
    dQtoR(q, R);
    dMassSetCylinderTotal(&back_link_4_mass, 10, 2, 2.0, 80);
    dMassRotate(&back_link_4_mass, R);
    dBodySetMass(back_link_4_body, &back_link_4_mass);
    dBodySetLinearVel(back_link_4_body, 0.0, 0.0, 0.0);
    dBodySetRotation(back_link_4_body, R);

    back_link_4_geom = dCreateCylinder(global_helper->getSpace(), 2.0, 80.0);
    dGeomSetData(back_link_4_geom, (void *)"back_link_4");
    dGeomSetBody(back_link_4_geom, back_link_4_body);
    dGeomSetPosition(back_link_4_geom, root_position[0] + 240.0, root_position[1], root_position[2]);


    dJointID ball_joint_1_2 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(ball_joint_1_2, back_link_1_body, back_link_2_body);
    dJointSetBallAnchor (ball_joint_1_2, root_position[0] + 80.0, root_position[1], root_position[2]);

    dJointID ball_joint_2_3 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(ball_joint_2_3, back_link_2_body, back_link_3_body);
    dJointSetBallAnchor (ball_joint_2_3, root_position[0] + 160.0, root_position[1], root_position[2]);
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

dGeomID ODEBodies::getPlaneGeom(){
	return plane_geom;
}




