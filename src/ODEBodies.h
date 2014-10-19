#ifndef ODEBODIES_H
#define ODEBODIES_H

#include <GL/glut.h>
#include <ode/ode.h>
#include <iostream>
#include <cmath>
#include <vector>
#include "helper.h"

class ODEBodies
{
private:
	//Private Objects
	helper * global_helper;

    float * root_position;

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
    dBodyID front_left_foot_link_4_body;
    dBodyID front_right_foot_link_1_body;
    dBodyID front_right_foot_link_2_body;
    dBodyID front_right_foot_link_3_body;
    dBodyID front_right_foot_link_4_body;
    dBodyID back_left_foot_link_1_body;
    dBodyID back_left_foot_link_2_body;
    dBodyID back_left_foot_link_3_body;
    dBodyID back_left_foot_link_4_body;
    dBodyID back_right_foot_link_1_body;
    dBodyID back_right_foot_link_2_body;
    dBodyID back_right_foot_link_3_body;
    dBodyID back_right_foot_link_4_body;
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
    dGeomID front_left_foot_link_4_geom;
    dGeomID front_right_foot_link_1_geom;
    dGeomID front_right_foot_link_2_geom;
    dGeomID front_right_foot_link_3_geom;
    dGeomID front_right_foot_link_4_geom;
    dGeomID back_left_foot_link_1_geom;
    dGeomID back_left_foot_link_2_geom;
    dGeomID back_left_foot_link_3_geom;
    dGeomID back_left_foot_link_4_geom;
    dGeomID back_right_foot_link_1_geom;
    dGeomID back_right_foot_link_2_geom;
    dGeomID back_right_foot_link_3_geom;
    dGeomID back_right_foot_link_4_geom;
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
    dMass front_left_foot_link_4_mass;
    dMass front_right_foot_link_1_mass;
    dMass front_right_foot_link_2_mass;
    dMass front_right_foot_link_3_mass;
    dMass front_right_foot_link_4_mass;
    dMass back_left_foot_link_1_mass;
    dMass back_left_foot_link_2_mass;
    dMass back_left_foot_link_3_mass;
    dMass back_left_foot_link_4_mass;
    dMass back_right_foot_link_1_mass;
    dMass back_right_foot_link_2_mass;
    dMass back_right_foot_link_3_mass;
    dMass back_right_foot_link_4_mass;
	dMass ball_mass;

    dReal back_link_1_length;
    dReal back_link_2_length;
    dReal back_link_3_length;
    dReal back_link_4_length;
    dReal back_link_5_length;
    dReal back_link_6_length;
    dReal front_foot_link_1_length;
    dReal front_foot_link_2_length;
    dReal front_foot_link_3_length;
    dReal front_foot_link_4_length;
    dReal back_foot_link_1_length;
    dReal back_foot_link_2_length;
    dReal back_foot_link_3_length;
    dReal back_foot_link_4_length;

    float back_link_1_theta;
    float back_link_2_theta;
    float back_link_3_theta;
    float back_link_4_theta;
    float back_link_5_theta;
    float back_link_6_theta;
    float front_foot_link_1_theta;
    float front_foot_link_2_theta;
    float front_foot_link_3_theta;
    float front_foot_link_4_theta;
    float back_foot_link_1_theta;
    float back_foot_link_2_theta;
    float back_foot_link_3_theta;
    float back_foot_link_4_theta;

	//Private Methods
    void set_ball();
    void set_back();
    void set_nnh();
    void set_tail();
    void set_front_legs();
    void set_back_legs();
    void set_leg();
    void set_plane();
    void extendMatrixTo4x3(dReal * inp_R, dReal * outp_R);
    void multiplyMatrices(float* matC, float* matA, float* matB, int rC, int cC, int rA, int cA, int rB, int cB);
    void setRotationMatrixZAxis(dReal * R, float theta);
    void setRotationMatrixXAxis(dReal * R, float theta);
    void setLink(dBodyID * link_body, dMass *link_mass, dReal link_length, float z_rotation_angle, dGeomID *link_geom, float * position);
public:
	//Constructor
    ODEBodies(helper * global_helper, float * root_position);

    //Public Methods
    void init();
    void step();

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
    dBodyID getFrontLeftFootLink4Body();
    //Front right foot    
    dBodyID getFrontRightFootLink1Body();
    dBodyID getFrontRightFootLink2Body();
    dBodyID getFrontRightFootLink3Body();
    dBodyID getFrontRightFootLink4Body();
    //Back left foot
    dBodyID getBackLeftFootLink1Body();
    dBodyID getBackLeftFootLink2Body();
    dBodyID getBackLeftFootLink3Body();
    dBodyID getBackLeftFootLink4Body();
    //Back right foot    
    dBodyID getBackRightFootLink1Body();
    dBodyID getBackRightFootLink2Body();
    dBodyID getBackRightFootLink3Body();
    dBodyID getBackRightFootLink4Body();

    float getBackLink1Length();
    float getBackLink2Length();
    float getBackLink3Length();
    float getBackLink4Length();
    float getBackLink5Length();
    float getBackLink6Length();

    float getFrontFootLink1Length();
    float getFrontFootLink2Length();
    float getFrontFootLink3Length();
    float getFrontFootLink4Length();

    float getBackFootLink1Length();
    float getBackFootLink2Length();
    float getBackFootLink3Length();
    float getBackFootLink4Length();

    dGeomID getPlaneGeom();
    helper * getGlobalHelper();

    vector<float> getLengths(int leg_id);
    vector<float> getAngles(int leg_id);
};

#endif // ODEBODIES_H
