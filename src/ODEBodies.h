#ifndef ODEBODIES_H
#define ODEBODIES_H

#include <GL/glut.h>
#include <ode/ode.h>
#include <iostream>
#include "helper.h"

class ODEBodies
{
private:
	//Private Objects
	helper * global_helper;
	dBodyID ball_body;
	dGeomID ball_geom;
	dGeomID plane_geom;
	dMass ball_mass;

	//Private Methods
    void set_ball();
    void set_plane();

public:
	//Constructor
    ODEBodies(helper * global_helper);

    //Public Methods
    void init();

    //Getters
    dBodyID getBallBody();
    dGeomID getBallGeom();
    dMass getBallMass();
    dGeomID getPlaneGeom();
};

#endif // ODEBODIES_H
