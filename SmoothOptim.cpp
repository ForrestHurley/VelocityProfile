#include "SmoothOptim.h"



void SmoothOptim::generateProfile(Path & in)
{
}

SmoothOptim::SmoothOptim()
{
}


SmoothOptim::~SmoothOptim()
{
}

Path SmoothOptim::initProfile()
{
	return Path();
}

float SmoothOptim::maxVelProf(float currIter)
{
	return 0.0f;
}

float SmoothOptim::maxAccProf(float currIter)
{
	return 0.0f;
}

float SmoothOptim::getMaxAccVelocity(float currIter, float lastIter)
{
	return 0.0f;
}

Path SmoothOptim::smoothJerk(float currIter)
{
	return Path();
}
