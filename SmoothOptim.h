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
	float maxVel(float currIter);
	float maxAcc(float currIter);
	float getMaxAccVelocity(float currIter, float lastIter);
	Path smoothJerk(float currIter);
};

