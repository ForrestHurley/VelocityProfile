#pragma once
#include "Optimizer.h"
class StandOptim :
	public Optimizer
{
public:

	float maximumAcceleration;
	float maximumJerk;

	StandOptim();
	~StandOptim();

private:

	//Override virtual functions from optimizer to add kinematic functionality

	float getSign(float in) {
		if (in > 0)
			return 1;
		else if (in < 0)
			return -1;
		else
			return 0;
	}

	bounds backCheck(Path *in, int iter, int forward);
	bounds foreCheck(Path *in, int iter, int forward);
	bounds staticCheck(Path *in, int iter, int forward);

	bounds minAccBound(Path *in, int iter, int forward);
	bounds minJerkBound(Path *in, int iter, int forward);

	bounds maxAccelerationKin(Path * in, int iter, int forward);
	bounds maxJerkKin(Path *in, int iter, int forward);

	//Estimates the delta time based on arc length and velocity
	float estimVelTime(Path *in, int iter, int forward);
	//Estimates the maximum acceleration vector magnitude based on delta time and maximum acceleration
	float getMaxAccMagn(Path *in, int iter, int forward);

	float getMaxJerkMagn(Path *in, int iter, int forward);
	//Gets delta arc length from the previously calculated path absolute arc lengths
	float getArcLength(Path *in, int iter, int forward);

};

