#ifndef HELPER_H
#define HELPER_H

#include <GL/glut.h>
#include <ode/ode.h>
#include <iostream>

using namespace std;

class helper
{
private:
	//Private Objects
	dWorldID world;
	dSpaceID space;
	dJointGroupID cgroup;

	//Private Methods
    void initWorld();
    void initSpace();
    void initCgroup();

public:
	//Constructor
    helper();

    //Public Methods
    void init();

    //Getters
    dWorldID getWorld();
    dSpaceID getSpace();
    dJointGroupID getCgroup();

};

#endif // HELPER_H
