#pragma once
#include "Optimizer.h"
class StandOptim :
	public Optimizer
{
public:

	float maximumAcceleration;

	StandOptim();
	~StandOptim();

private:

	//Override virtual functions from optimizer to add kinematic functionality

	bounds backCheck(Path *in, int iter, int forward);
	bounds foreCheck(Path *in, int iter, int forward);
	bounds staticCheck(Path *in, int iter, int forward);

	//Estimates the delta time based on arc length and velocity
	float estimVelTime(Path *in, int iter, int forward);
	//Estimates the maximum acceleration vector magnitude based on delta time and maximum acceleration
	float getMaxAccMagn(Path *in, int iter, int forward);
	//Gets delta arc length from the previously calculated path absolute arc lengths
	float getArcLength(Path *in, int iter, int forward);

};

