#ifndef HELPER_H
#define HELPER_H

#include <GL/glut.h>
#include <ode/ode.h>
#include <iostream>

class helper
{
private:
	dWorldID world;
	dBodyID ball_body;
	dSpaceID space;
	dGeomID ball_geom;
	dGeomID plane_geom;
	dMass ball_mass;
	dJointGroupID cgroup;

public:
    helper();
    void initWorld();
    dWorldID getWorld();
};

#endif // HELPER_H
