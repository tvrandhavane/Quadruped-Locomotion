#ifndef ODEBODIES_H
#define ODEBODIES_H

#include <GL/glut.h>
#include <ode/ode.h>
#include <iostream>
#include <cmath>
#include "helper.h"
#include "controller.h"

class ODEBodies
{
private:
	//Private Objects
	helper * global_helper;
    controller * gait_controller;

    float root_position[3];

    dBodyID back_link_1_body;
    dBodyID back_link_2_body;
    dBodyID back_link_3_body;
    dBodyID back_link_4_body;
    dBodyID back_link_5_body;
    dBodyID back_link_6_body;
    dBodyID nnh_link_1_body;
    dBodyID nnh_link_2_body;
    dBodyID nnh_link_3_body;
    dBodyID nnh_link_4_body;
    dBodyID tail_link_1_body;
    dBodyID tail_link_2_body;
    dBodyID tail_link_3_body;
    dBodyID tail_link_4_body;
    dBodyID front_left_foot_link_1_body;
    dBodyID front_left_foot_link_2_body;
    dBodyID front_left_foot_link_3_body;
    dBodyID front_right_foot_link_1_body;
    dBodyID front_right_foot_link_2_body;
    dBodyID front_right_foot_link_3_body;
    dBodyID back_left_foot_link_1_body;
    dBodyID back_left_foot_link_2_body;
    dBodyID back_left_foot_link_3_body;
    dBodyID back_right_foot_link_1_body;
    dBodyID back_right_foot_link_2_body;
    dBodyID back_right_foot_link_3_body;
	dBodyID ball_body;

    dGeomID back_link_1_geom;
    dGeomID back_link_2_geom;
    dGeomID back_link_3_geom;
    dGeomID back_link_4_geom;
    dGeomID back_link_5_geom;
    dGeomID back_link_6_geom;
    dGeomID nnh_link_1_geom;
    dGeomID nnh_link_2_geom;
    dGeomID nnh_link_3_geom;
    dGeomID nnh_link_4_geom;
    dGeomID tail_link_1_geom;
    dGeomID tail_link_2_geom;
    dGeomID tail_link_3_geom;
    dGeomID tail_link_4_geom;
    dGeomID front_left_foot_link_1_geom;
    dGeomID front_left_foot_link_2_geom;
    dGeomID front_left_foot_link_3_geom;
    dGeomID front_right_foot_link_1_geom;
    dGeomID front_right_foot_link_2_geom;
    dGeomID front_right_foot_link_3_geom;
    dGeomID back_left_foot_link_1_geom;
    dGeomID back_left_foot_link_2_geom;
    dGeomID back_left_foot_link_3_geom;
    dGeomID back_right_foot_link_1_geom;
    dGeomID back_right_foot_link_2_geom;
    dGeomID back_right_foot_link_3_geom;
	dGeomID ball_geom;
	dGeomID plane_geom;

    dMass back_link_1_mass;
    dMass back_link_2_mass;
    dMass back_link_3_mass;
    dMass back_link_4_mass;
    dMass back_link_5_mass;
    dMass back_link_6_mass;
    dMass nnh_link_1_mass;
    dMass nnh_link_2_mass;
    dMass nnh_link_3_mass;
    dMass nnh_link_4_mass;
    dMass tail_link_1_mass;
    dMass tail_link_2_mass;
    dMass tail_link_3_mass;
    dMass tail_link_4_mass;
    dMass front_left_foot_link_1_mass;
    dMass front_left_foot_link_2_mass;
    dMass front_left_foot_link_3_mass;
    dMass front_right_foot_link_1_mass;
    dMass front_right_foot_link_2_mass;
    dMass front_right_foot_link_3_mass;
    dMass back_left_foot_link_1_mass;
    dMass back_left_foot_link_2_mass;
    dMass back_left_foot_link_3_mass;
    dMass back_right_foot_link_1_mass;
    dMass back_right_foot_link_2_mass;
    dMass back_right_foot_link_3_mass;
	dMass ball_mass;

    float back_link_4_theta;
    float back_link_6_theta;

    dReal front_foot_link_1_length;
    dReal front_foot_link_2_length;
    dReal front_foot_link_3_length;

    dReal back_foot_link_1_length;
    dReal back_foot_link_2_length;
    dReal back_foot_link_3_length;

	//Private Methods
    void set_ball();
    void set_back();
    void set_nnh();
    void set_tail();
    void set_front_legs();
    void set_back_legs();
    void set_leg();
    void set_plane();
    void setRotationMatrixZAxis(dReal * R, float theta);

public:
	//Constructor
    ODEBodies(helper * global_helper, controller * gait_controller);

    //Public Methods
    void init();

    //Getters
    dBodyID getBallBody();
    //Back
    dBodyID getBackLink1Body();
    dBodyID getBackLink2Body();
    dBodyID getBackLink3Body();
    dBodyID getBackLink4Body();
    dBodyID getBackLink5Body();
    dBodyID getBackLink6Body();
    //Neck and head
    dBodyID getNnhLink1Body();
    dBodyID getNnhLink2Body();
    dBodyID getNnhLink3Body();
    dBodyID getNnhLink4Body();
    //Tail
    dBodyID getTailLink1Body();
    dBodyID getTailLink2Body();
    dBodyID getTailLink3Body();
    dBodyID getTailLink4Body();
    //Front left foot
    dBodyID getFrontLeftFootLink1Body();
    dBodyID getFrontLeftFootLink2Body();
    dBodyID getFrontLeftFootLink3Body();
    //Front right foot    
    dBodyID getFrontRightFootLink1Body();
    dBodyID getFrontRightFootLink2Body();
    dBodyID getFrontRightFootLink3Body();
    //Back left foot
    dBodyID getBackLeftFootLink1Body();
    dBodyID getBackLeftFootLink2Body();
    dBodyID getBackLeftFootLink3Body();
    //Back right foot    
    dBodyID getBackRightFootLink1Body();
    dBodyID getBackRightFootLink2Body();
    dBodyID getBackRightFootLink3Body();

    float getFrontFootLink1Length();
    float getFrontFootLink2Length();
    float getFrontFootLink3Length();

    float getBackFootLink1Length();
    float getBackFootLink2Length();
    float getBackFootLink3Length();

    dGeomID getPlaneGeom();
    helper * getGlobalHelper();
    controller * getGaitController();
};

#endif // ODEBODIES_H
