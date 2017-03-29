#include "VectorOptim.h"



void VectorOptim::generateProfile(Path *in)
{
	for (int i = 1; i < in->path.size(); i++) {
		in->path[i].velocity = initMaxVelInDir(i, i-1, in);
	}
	for (int i = 1; i < in->path.size(); i++) {
		in->path[i].velocity = iterMaxVelInDir(i, i - 1, in);
	}
	for (int i = in->path.size()-1; i > 0; i--) {
		in->path[i].velocity = iterMaxVelInDir(i, i + 1, in);
	}
}

Vect VectorOptim::initMaxVelInDir(int iter, int lastIter, Path * in)
{
	Vect out = in->path[iter].velocity.normalize();
	Vect lastDir = in->path[lastIter].velocity.normalize();

	out = out * sqrtf(maxAcceleration*(in->path[iter].arclen - in->path[lastIter].arclen) / acosf(out.dot(lastDir)));

	return out;
}

Vect VectorOptim::iterMaxVelInDir(int iter, int lastIter, Path * in)
{
	float before = in->path[lastIter].velocity.magnitude();
	Vect direction = in->path[iter].velocity.normalize();
	Vect vPlusA = in->path[lastIter].acceleration + in->path[lastIter].velocity;
	float dirDot = direction.dot(vPlusA);
	float maxJ = jerkMagn(iter, lastIter, in);
	float after = dirDot + sqrtf(dirDot*dirDot + maxJ*maxJ - vPlusA.dot(vPlusA));
	Vect out = direction * after;
	return out;
}

float VectorOptim::jerkMagn(int iter, int lastIter, Path * in)
{
	float termA = 2 * maxJerk * (in->path[lastIter].velocity.magnitude() - in->path[iter].arclen + in->path[lastIter].arclen);
	float lastAccel = in->path[lastIter].acceleration.magnitude();
	float out = termA / (lastAccel + sqrtf(lastAccel*lastAccel - termA));
	return out;
}

VectorOptim::VectorOptim()
{
}


VectorOptim::~VectorOptim()
{
}
