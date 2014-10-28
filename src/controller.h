#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
#include <cmath>
#include "inverseKinematics.h"
#include "ODEBodies.h"

#define g 9800

using namespace std;

class controller
{
private:
	//Private Objects
    int time;
    int T;
    int swingStart[4];
    int swingEnd[4];

    int shoulderHeight;
    int hipHeight;

    float lfHeight[4];

    float current_velocity[3];
    float desired_velocity[3];

    float prev_stepping_location[4][3];
    float next_stepping_location[4][3];
    float current_foot_location[4][3];

    float foot_link_pd_error[4][4];
    float foot_link_gain_kp[4][4];
    float foot_link_gain_kd[4][4];

    float swing_torque[4][3];
    bool swingFlag[4];

    float * root_position;

    ODEBodies * body_bag;

	//Private Methods
    bool isInSwing(int leg_id);
    void computePlacementLocation(int leg_id, float h);
    void setFootLocation(int leg_id, int phase);
    void stanceLegTreatment(int leg_id);
    float computeSwingPhase(int leg_id, int phase);
    void legController(int leg_id, int phase);
    vector<float> getTargetPosition(int leg_id);
public:
    //Constructor
    controller(ODEBodies * body_bag, float * root_position);
    
    //Public Methods
    void takeStep();
    Eigen::MatrixXf applyIK(vector<float> lengths, vector<Eigen::MatrixXf> transformationMatrices, vector<float> angles, vector<float> endEffector, Eigen::MatrixXf translationMatrix, Eigen::MatrixXf axis);
    void gravityCompensation();
};

#endif // CONTROLLER_H
