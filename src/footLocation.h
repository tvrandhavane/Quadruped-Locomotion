#ifndef FOOTLOCATION_H
#define FOOTLOCATION_H

#include <iostream>
#include <cmath>

#define g 980

using namespace std;

class footLocation
{
private:
	//Private Objects
    float placementLocation[3]; 

	//Private Methods
    

public:
    //Constructor
    footLocation();
    
    //Public Methods
    void computePlacementLocation(int h, float current_velocity[3], float desired_velocity[3]);
};

#endif // FOOTLOCATION_H
