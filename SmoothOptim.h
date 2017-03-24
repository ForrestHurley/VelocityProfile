#pragma once
#include "Optimizer.h"
class SmoothOptim :
	public Optimizer
{
public:
	float maxVel;
	float maxAcc;
	float maxJerk;

	void generateProfile(Path &in);

	SmoothOptim();
	~SmoothOptim();

private:

	Path initProfile();
	float maxVelProf(float currIter);
	float maxAccProf(float currIter);
	float getMaxAccVelocity(float currIter, float lastIter);
	Path smoothJerk(float currIter);
};

