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
    dMassSetSphereTotal(&ball_mass, 0.1, 0.2);
    dBodySetMass(ball_body, &ball_mass);
    dBodySetLinearVel(ball_body, -7.0, 0.0, 0.0);

    ball_geom = dCreateSphere(global_helper->getSpace(), 10.0);
    dGeomSetData(ball_geom, (void *)"ball");
    dGeomSetBody(ball_geom, ball_body);
    dGeomSetPosition(ball_geom, 0.0, 0.0, 0.0);
}

void ODEBodies::set_back(){
    float root_position[3];

    root_position[0] = 30.0;
    root_position[1] = 35.0;
    root_position[2] = 0.0;
    
    dQuaternion q;
    q[0] = 3.14/2;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 1.0;
    back_link_1_body = dBodyCreate(global_helper->getWorld());
    dMassSetZero(&back_link_1_mass);
    dMassSetSphereTotal(&back_link_1_mass, 0.1, 0.2);
    dBodySetMass(back_link_1_body, &back_link_1_mass);
    dBodySetLinearVel(back_link_1_body, 0.0, 0.0, 0.0);

    back_link_1_geom = dCreateCylinder(global_helper->getSpace(), 0.2, 1.0);
    dGeomSetData(back_link_1_geom, (void *)"back_link_1");
    dGeomSetBody(back_link_1_geom, back_link_1_body);
    dGeomSetPosition(back_link_1_geom, root_position[0], root_position[1], root_position[2]);
    dBodySetQuaternion(back_link_1_body, q);

    back_link_2_body = dBodyCreate(global_helper->getWorld());
    dMassSetZero(&back_link_2_mass);
    dMassSetSphereTotal(&back_link_2_mass, 0.1, 0.2);
    dBodySetMass(back_link_2_body, &back_link_2_mass);
    dBodySetLinearVel(back_link_2_body, 0.0, 0.0, 0.0);

    back_link_2_geom = dCreateCylinder(global_helper->getSpace(), 0.2, 1.0);
    dGeomSetData(back_link_2_geom, (void *)"back_link_2");
    dGeomSetBody(back_link_2_geom, back_link_2_body);
    dGeomSetPosition(back_link_2_geom, root_position[0] + 60.0, root_position[1], root_position[2]);
    dBodySetQuaternion(back_link_2_body, q);

    back_link_3_body = dBodyCreate(global_helper->getWorld());
    dMassSetZero(&back_link_3_mass);
    dMassSetSphereTotal(&back_link_3_mass, 0.1, 0.2);
    dBodySetMass(back_link_3_body, &back_link_3_mass);
    dBodySetLinearVel(back_link_3_body, 0.0, 0.0, 0.0);

    back_link_3_geom = dCreateCylinder(global_helper->getSpace(), 0.2, 1.0);
    dGeomSetData(back_link_3_geom, (void *)"back_link_3");
    dGeomSetBody(back_link_3_geom, back_link_3_body);
    dGeomSetPosition(back_link_3_geom, root_position[0] + 120.0, root_position[1], root_position[2]);
    dBodySetQuaternion(back_link_2_body, q);

    dJointID ball_joint = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(ball_joint, back_link_1_body, back_link_2_body);
    dJointSetBallAnchor (ball_joint, root_position[0] + 30.0, root_position[1], root_position[2]);
}

void ODEBodies::set_plane(){
	plane_geom = dCreatePlane(global_helper->getSpace(), 0.0, 1.0, 0.0, -200.0);	
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

dGeomID ODEBodies::getPlaneGeom(){
	return plane_geom;
}




