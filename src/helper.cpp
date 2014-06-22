#include "helper.h"

helper::helper(){
}

void helper::initWorld(){
	world = dWorldCreate();
    dWorldSetGravity(world, 0.0, 10.0, 0.0);
    dWorldSetERP(world, 0.9);
    dWorldSetCFM(world, 1e-4);
    dWorldSetLinearDamping(world, 0.00001);
    dWorldSetAngularDamping(world, 0.005);
    dWorldSetMaxAngularSpeed(world, 200);

    dWorldSetContactMaxCorrectingVel (world,0.1);
    dWorldSetContactSurfaceLayer (world,0.1);
}

void helper::initSpace(){
	space = dSimpleSpaceCreate(0);
}

void helper::initCgroup(){
	cgroup = dJointGroupCreate(0);
}

dWorldID helper::getWorld(){
	return world;
}

dSpaceID helper::getSpace(){
	return space;
}

dJointGroupID helper::getCgroup(){
	return cgroup;
}