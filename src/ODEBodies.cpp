#include "ODEBodies.h"

ODEBodies::ODEBodies(helper * global_helper){
	this->global_helper = global_helper;
}

void ODEBodies::init(){
	set_ball();
	set_plane();
}

void ODEBodies::set_ball(){
	ball_body = dBodyCreate(global_helper->getWorld());
    dMassSetZero(&ball_mass);
    dMassSetSphereTotal(&ball_mass, 0.1, 0.2);
    dBodySetMass(ball_body, &ball_mass);
    dBodySetLinearVel(ball_body, -7.0, 0.0, 0.0);

    ball_geom = dCreateSphere(global_helper->getSpace(), 0.2);
    dGeomSetData(ball_geom, (void *)"ball");
    dGeomSetBody(ball_geom, ball_body);
    dGeomSetPosition(ball_geom, 0.0, 200.0, 0.0);
}

void ODEBodies::set_plane(){
	plane_geom = dCreatePlane(global_helper->getSpace(), 0.0, 1.0, 0.0, -390.0);	
}

dBodyID ODEBodies::getBallBody(){
	return ball_body;
}

dGeomID ODEBodies::getPlaneGeom(){
	return plane_geom;
}




