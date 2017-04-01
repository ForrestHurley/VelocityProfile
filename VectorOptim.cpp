#include "VectorOptim.h"
#include <iostream>


void VectorOptim::generateProfile(Path *in)
{
	
}


VectorOptim::VectorOptim()
{
}


VectorOptim::~VectorOptim()
{
}

void VectorOptim::initMaxVel(Path * in)
{
	for (int i = 0; i < in->path.size(); i++) {
		in->path[i].velocity = in->path[i].velocity.normalize()*(maximumVelocity + 1);
	}
}

void VectorOptim::recurOptim(Path * in, int iter, int forward, bounds(*backCheck)(Path *in, int iter, bool forward), bounds(*foreCheck)(Path *in, int iter, bool forward))
{

	bounds lastBounds;
	bounds nextBounds;
	bounds finalBounds;

	while (true) {

		lastBounds = backCheck(in, iter, forward);
		nextBounds = foreCheck(in, iter, forward);

		finalBounds = lastBounds;
		finalBounds.combine(nextBounds);

		if (finalBounds.null()) {
			recurOptim(in, iter, -forward, backCheck, foreCheck);
		}
		
		if (finalBounds.max > in->path[iter].velocity.magnitude()) {
			
			break;
		}

		iter += forward;
		if ((iter < 0) || (iter > in->path.size())) {
			
			break;
		}

	}

}

VectorOptim::bounds VectorOptim::maxJerkKin(Path * in, int iter, int forward)
{
	Vect lastVel = in->path[iter - forward].velocity;
	Vect lastAcc = in->path[iter - forward].acceleration;
	Vect lastSum = lastVel + lastAcc;
	Vect dir = in->path[iter].velocity.normalize();
	float dot = dir.dot(lastSum);
	float maxJerk = getMaxJerkMagn(in, iter, forward);
	float descrim = sqrtf(dot*dot + maxJerk*maxJerk -lastSum.dot(lastSum));
	bounds out;
	out.min = dot - descrim;
	out.max = dot + descrim;
	return out;
}

VectorOptim::bounds VectorOptim::maxAccelerationKin(Path * in, int iter, int forward)
{
	Vect lastVel = in->path[iter-forward].velocity;
	Vect dir = in->path[iter].velocity.normalize();
	float dot = dir.dot(lastVel);
	float maxAcc = getMaxAccMagn(in, iter, forward);
	float descrim = sqrtf(dot*dot + maxAcc*maxAcc - lastVel.dot(lastVel));
	bounds out;
	out.min = dot - descrim;
	out.max = dot + descrim;
	return out;
}

VectorOptim::bounds VectorOptim::maxVelocityKin(Path * in, int iter, int forward)
{
	return bounds(0, maximumVelocity);
}

VectorOptim::bounds VectorOptim::kinematicBoundsKin(Path * in, int iter, int forward)
{
	bounds out = maxVelocityKin(in, iter, forward);
	out.combine(maxAccelerationKin(in, iter, forward));
	out.combine(maxJerkKin(in, iter, forward));
	return out;
}

VectorOptim::bounds VectorOptim::maxJerk(Path * in, int iter, int forward)
{
	float accel = in->path[iter - forward].acceleration.magnitude();
	float sine = in->path[iter].acceleration.sine(in->path[iter].velocity);
	float max = 
	return bounds();
}

VectorOptim::bounds VectorOptim::maxAcceleration(Path * in, int iter, int forward)
{
	float length = getArcLength(in, iter, forward);
	float max = sqrtf(maximumAcceleration*length / in->path[iter].acceleration.sine(in->path[iter].velocity));
	
	bounds out(0, max);
	return out;
}

VectorOptim::bounds VectorOptim::kinematicBounds(Path * in, int iter, int forward)
{
	bounds out = maxAcceleration(in, iter, forward);
	out.combine(maxJerk(in, iter, forward));
	return out;
}

float VectorOptim::estimVelTime(Path * in, int iter, int forward)
{
	return getArcLength(in,iter,forward)/in->path[iter-forward].velocity.magnitude();
}

float VectorOptim::getMaxAccMagn(Path * in, int iter, int forward)
{
	return estimVelTime(in, iter, forward)*maximumAcceleration;
}

float VectorOptim::getMaxJerkMagn(Path * in, int iter, int forward)
{
	return estimVelTime(in, iter, forward)*maximumJerk;
}

float VectorOptim::getArcLength(Path * in, int iter, int forward)
{
	return abs(in->path[iter].arclen - in->path[iter - forward].arclen);
}
