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
};

