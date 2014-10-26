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
    void applyIK(vector<float> lengths, vector<float> angles, vector<float> endEffector, QSMatrix<float> transformationMatrix, QSMatrix<float> axis);
    void gravityCompensation();
};

#endif // CONTROLLER_H
