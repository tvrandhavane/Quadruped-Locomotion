#include "footLocation.h"

footLocation::footLocation(){

}

void footLocation::computePlacementLocation(int h, float current_velocity[3], float desired_velocity[3]){
	float df[3];

	float velocitySqr = desired_velocity[0]*desired_velocity[0];
	velocitySqr += desired_velocity[1]*desired_velocity[1];
	velocitySqr += desired_velocity[2]*desired_velocity[2];

	float dfFactor = sqrt((h/g) + (velocitySqr/(4*g*g)));

	df[0] = desired_velocity[0] * dfFactor;
	df[1] = desired_velocity[1] * dfFactor;
	df[2] = desired_velocity[2] * dfFactor;

	float targetLocation[3];
	targetLocation[0] = df[0] + (current_velocity[0] - desired_velocity[0])*sqrt(h/g);
	targetLocation[1] = df[1] + (current_velocity[1] - desired_velocity[1])*sqrt(h/g);
	targetLocation[2] = df[2] + (current_velocity[2] - desired_velocity[2])*sqrt(h/g);

	//cout << "TargetLocation = " << targetLocation[0] << " " << targetLocation[1] << " " << 
	//	targetLocation[2] << endl;
}