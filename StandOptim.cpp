#include "StandOptim.h"



StandOptim::StandOptim()
{
}


StandOptim::~StandOptim()
{
}

Optimizer::bounds StandOptim::backCheck(Path * in, int iter, int forward)
{
	bounds out = bounds(in->path[iter - forward].velocity.magnitude() - getMaxAccMagn(in, iter, forward), in->path[iter - forward].velocity.magnitude() + getMaxAccMagn(in, iter, forward));
	out.combine(maxAccelerationKin(in, iter, forward));
	out.combine(maxJerkKin(in, iter, forward));
	//std::cout << "Max Acc: " << maxAccelerationKin(in, iter, forward).min << " Max Jerk: " << maxJerkKin(in, iter, forward).min << std::endl;
	return out;
}

Optimizer::bounds StandOptim::foreCheck(Path * in, int iter, int forward)
{
	bounds out = bounds(0, maximumVelocity);
	out.combine(maxJerkKin(in, iter, forward));
	out.combine(maxJerkKin(in, iter, -forward));
	return out;
}

Optimizer::bounds StandOptim::staticCheck(Path * in, int iter, int forward)
{
	bounds out = bounds(0, maximumVelocity);

	out.combine(minAccBound(in, iter, forward));
	out.combine(minJerkBound(in, iter, forward));
	out.combine(minJerkBound(in, iter, -forward));

	return out;
}

Optimizer::bounds StandOptim::minAccBound(Path * in, int iter, int forward)
{
	Vect lastVel;
	if ((forward == 1 && iter > 0) || (forward == -1 && iter < in->path.size() - 1))
		lastVel = in->path[iter - forward].velocity;
	else
		lastVel = in->path[iter].velocity;
	Vect dir = in->path[iter].velocity.normalize();
	float dot = abs(lastVel.dot(dir));
	float cross = abs(lastVel.cross(dir));

	if (lastVel.magnitude() > maximumVelocity) {
		return bounds();
	}

	if (verbose)
		if (isnan(getMaxAccMagn(in, iter, forward)*dot / cross))
			std::cout << "Null Values, Dot: " << dot << " Cross: " << cross << std::endl;
	bounds out;
	if (cross != 0 && dot != 0)
		out.combine(bounds(0, getMaxAccMagn(in, iter, forward)*dot / cross));

	return out;
}

Optimizer::bounds StandOptim::minJerkBound(Path * in, int iter, int forward)
{
	Vect lastVel;
	if ((forward == 1 && iter > 0) || (forward == -1 && iter < in->path.size() - 1))
		lastVel = in->path[iter - forward].velocity;
	else
		lastVel = in->path[iter].velocity;
	Vect fullVel = in->path[iter].velocity;
	Vect vel = fullVel.normalize();

	Vect beforeLastVel;

	float newAcc;

	if ((forward == 1 && iter > 1) || (forward == -1 && iter < in->path.size() - 2)) {
		beforeLastVel = in->path[iter - 2 * forward].velocity;
		if (lastVel.magnitude() > maximumVelocity) {
			return bounds();
		}
		if (beforeLastVel.magnitude() > maximumVelocity)
			beforeLastVel = lastVel;

		float lastAcc = (lastVel - beforeLastVel).magnitude() / estimVelTime(in, iter - forward, forward);
		newAcc = lastAcc*estimVelTime(in, iter, forward) + getMaxJerkMagn(in, iter, forward);
	}
	else {
		beforeLastVel = lastVel;
		newAcc = getMaxJerkMagn(in, iter, forward);
	}

	float dot = abs(lastVel.dot(vel));
	float cross = abs(lastVel.cross(vel));

	if (verbose)
		if (isnan(newAcc * dot / cross))
			std::cout << "Jerk Null Values, Dot: " << dot << " Cross: " << cross << std::endl;
	bounds out;
	if (cross != 0 && dot != 0)
		out.combine(bounds(0, newAcc*dot / cross));

	return out;
}

Optimizer::bounds StandOptim::maxAccelerationKin(Path * in, int iter, int forward)
{
	Vect lastVel;
	if ((forward == 1 && iter > 0) || (forward == -1 && iter < in->path.size() - 1))
		lastVel = in->path[iter - forward].velocity;
	else
		lastVel = in->path[iter].velocity;
	Vect dir = in->path[iter].velocity.normalize();
	if (lastVel.magnitude() > maximumVelocity) {
		return bounds();
	}
	float dot = lastVel.dot(dir);
	float cross = lastVel.cross(dir);
	float maxAcc = getMaxAccMagn(in, iter, forward);
	float descrim = sqrtf(maxAcc*maxAcc-cross*cross);
	bounds out;
	out.min = dot - descrim;
	out.max = dot + descrim;
	return out;
}

Optimizer::bounds StandOptim::maxJerkKin(Path * in, int iter, int forward)
{
	Vect lastVel;
	if ((forward == 1 && iter > 0) || (forward == -1 && iter < in->path.size() - 1))
		lastVel = in->path[iter - forward].velocity;
	else
		lastVel = in->path[iter].velocity;
	Vect fullVel = in->path[iter].velocity;
	Vect vel = fullVel.normalize();

	Vect beforeLastVel;

	float newAcc;
	float newAccA;
	float newAccB;

	if ((forward == 1 && iter > 1) || (forward == -1 && iter < in->path.size() - 2)) {
		beforeLastVel = in->path[iter - 2 * forward].velocity;
		//std::cout << "Jerk Last Vel: " << lastVel.magnitude() << " Before Last Vel: " << beforeLastVel.magnitude() << std::endl;
		if (lastVel.magnitude() > maximumVelocity) {
			return bounds();
		}
		if (beforeLastVel.magnitude() > maximumVelocity)
			beforeLastVel = lastVel;

		float lastAcc = (lastVel - beforeLastVel).magnitude() / estimVelTime(in, iter - forward, forward);
		newAccA = lastAcc*estimVelTime(in,iter,forward) + getMaxJerkMagn(in, iter, forward);
		newAccB = lastAcc*estimVelTime(in, iter, forward) - getMaxJerkMagn(in, iter, forward);
	}
	else {
		newAcc = getMaxJerkMagn(in, iter, forward);
		float dot = lastVel.dot(vel);
		float cross = lastVel.cross(vel);
		float descrim = sqrtf(newAcc*newAcc - cross*cross);
		bounds out;
		out.min = dot - descrim;
		out.max = dot + descrim;
		return out;
	}

	float dot = lastVel.dot(vel);
	float cross = lastVel.cross(vel);
	float descrimA = getSign(newAccA)*sqrtf(newAccA*newAccA - cross*cross);
	float descrimB = getSign(newAccB)*sqrtf(newAccB*newAccB - cross*cross);
	bounds out;
	out.min = dot + descrimB;
	out.max = dot + descrimA;
	return out;
}

float StandOptim::estimVelTime(Path * in, int iter, int forward)
{
	return getArcLength(in, iter, forward) / maximumVelocity;
}

float StandOptim::getMaxAccMagn(Path * in, int iter, int forward)
{
	return estimVelTime(in, iter, forward)*maximumAcceleration;
}

float StandOptim::getMaxJerkMagn(Path * in, int iter, int forward)
{
	return estimVelTime(in,iter,forward)*maximumJerk;
}

float StandOptim::getArcLength(Path * in, int iter, int forward)
{
	return abs(in->path[iter].arclen - in->path[iter - forward].arclen);
}
