#include "VectorOptim.h"
#include <iostream>


VectorOptim::VectorOptim()
{
}


VectorOptim::~VectorOptim()
{
}

Optimizer::bounds VectorOptim::backCheck(Path * in, int iter, int forward)
{
	return kinematicBoundsKin(in, iter, forward);
}

Optimizer::bounds VectorOptim::foreCheck(Path * in, int iter, int forward)
{
	return kinematicBounds(in, iter, forward);
}

Optimizer::bounds VectorOptim::staticCheck(Path * in, int iter, int forward)
{
	return maxVelocity(in, iter, forward);
}

Optimizer::bounds VectorOptim::maxJerkKin(Path * in, int iter, int forward)
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

Optimizer::bounds VectorOptim::maxAccelerationKin(Path * in, int iter, int forward)
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

Optimizer::bounds VectorOptim::kinematicBoundsKin(Path * in, int iter, int forward)
{
	bounds out = maxAccelerationKin(in, iter, forward);
	//out.combine(maxJerkKin(in, iter, forward));
	return out;
}

Optimizer::bounds VectorOptim::maxJerk(Path * in, int iter, int forward)
{
	float accel = in->path[iter - forward].acceleration.magnitude();
	float sine = in->path[iter].acceleration.sine(in->path[iter].velocity);
	//float max = 
	return bounds();
}

Optimizer::bounds VectorOptim::maxAcceleration(Path * in, int iter, int forward)
{
	float length = getArcLength(in, iter, forward);
	float max = sqrtf(abs(maximumAcceleration*length / in->path[iter-forward].acceleration.sine(in->path[iter-forward].velocity)));
	if (isnan(max)) max = sqrtf(maximumAcceleration*length);
	//std::cout << "Max forward vel from acceleration: " << max << " Sine: " << in->path[iter-forward].acceleration.sine(in->path[iter-forward].velocity) << " Length: " << length << std::endl;
	bounds out(0, max);
	return out;
}

Optimizer::bounds VectorOptim::kinematicBounds(Path * in, int iter, int forward)
{
	bounds out = maxAcceleration(in, iter, forward);
	out.combine(maxJerk(in, iter, forward));
	return out;
}

Optimizer::bounds VectorOptim::maxVelocity(Path * in, int iter, int forward)
{
	return bounds(0, maximumVelocity);
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

void VectorOptim::updateParameters(Path * in, int iter, int forward)
{
	updateAcceleration(in, iter, forward);
}


void VectorOptim::updateAcceleration(Path * in, int iter, int forward)
{
	in->path[iter].acceleration.x = in->path[iter].velocity.x - in->path[iter-forward].velocity.x;
	in->path[iter].acceleration.y = in->path[iter].velocity.y - in->path[iter-forward].velocity.y;

	in->path[iter].acceleration = in->path[iter].acceleration / estimVelTime(in, iter, forward);
}
