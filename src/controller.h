#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
#include <cmath>

using namespace std;

#define g 9800

class controller
{
private:
	//Private Objects
    int time;
    int T;
    int swingStart[4];
    int swingEnd[4];

    float root_position[3];

    int shoulderHeight;
    int hipHeight;

    float lfHeight[4];

    float current_velocity[3];
    float desired_velocity[3];

    float prev_stepping_location[4][3];
    float next_stepping_location[4][3];
    float current_foot_location[4][3];

	//Private Methods
    bool isInSwing(int leg_id);
    void computePlacementLocation(int leg_id, float h);
    void setFootLocation(int leg_id, int phase);
    void stanceLegTreatment(int leg_id);
    float computeSwingPhase(int leg_id, int phase);

public:
    //Constructor
    controller();
    
    //Public Methods
    void takeStep();
    float * getRootPosition();

};

#endif // CONTROLLER_H
