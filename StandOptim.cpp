#include "StandOptim.h"



StandOptim::StandOptim()
{
}


StandOptim::~StandOptim()
{
}

Optimizer::bounds StandOptim::backCheck(Path * in, int iter, int forward)
{
	return bounds(in->path[iter-forward].velocity.magnitude()-getMaxAccMagn(in,iter,forward), in->path[iter - forward].velocity.magnitude() + getMaxAccMagn(in, iter, forward));
}

Optimizer::bounds StandOptim::foreCheck(Path * in, int iter, int forward)
{
	return bounds();
}

Optimizer::bounds StandOptim::staticCheck(Path * in, int iter, int forward)
{
	bounds out = bounds(0, maximumVelocity);
	out.combine(bounds(0, sqrtf(maximumAcceleration / in->path[iter].curvature)));
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

float StandOptim::getArcLength(Path * in, int iter, int forward)
{
	return abs(in->path[iter].arclen - in->path[iter - forward].arclen);
}
