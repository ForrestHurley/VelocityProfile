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
	Vect initMaxVelInDir(int iteration, Path *in);
	Vect iterMaxVelInDir(int iteration, Path *in);
	float jerkMagn(int iteration, Path *in);
};

