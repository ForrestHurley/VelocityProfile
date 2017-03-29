#pragma once
#include "Optimizer.h"
#include "Path.h"
class VectorOptim :
	public Optimizer
{
public:

	float maxVelocity;
	float maxAcceleration;
	float maxJerk;

	void generateProfile(Path *in);

	VectorOptim();
	~VectorOptim();
private:
	Vect initMaxVelInDir(int iter, int lastIter, Path *in);
	Vect iterMaxVelInDir(int iter, int lastIter, Path *in);
	float jerkMagn(int iter, int lastIter, Path *in);
};

