#include "VectorOptim.h"



void VectorOptim::generateProfile(Path *in)
{
	for (int i = 1; i < in->path.size(); i++) {
		in->path[i].velocity = initMaxVelInDir(i, in);
	}
	for (int i = 0; i < in->path.size() - 1; i++) {
		in->path[i].velocity = iterMaxVelInDir(i, in);
	}
}

Vect VectorOptim::initMaxVelInDir(int iteration, Path * in)
{
	Vect out = in->velAtParam(in->path[iteration].proportion).normalize();
	Vect nextDir = in->velAtParam(in->path[iteration].proportion).normalize();

	out = out * sqrtf(maxAcceleration*in->path[iteration - 1].arclen / acosf(out.dot(nextDir)));

	return out;
}

Vect VectorOptim::iterMaxVelInDir(int iteration, Path * in)
{
	float before = in->path[iteration+1].velocity.magnitude();
	Vect direction = in->path[iteration+1].velocity.normalize();
	float after = direction
	return Vect();
}

float VectorOptim::jerkMagn(int iteration, Path * in)
{
	return 0.0f;
}

VectorOptim::VectorOptim()
{
}


VectorOptim::~VectorOptim()
{
}
