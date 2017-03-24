#include "Path.h"

Path::Path(int len)
{
	numSteps = len;
	initPath();
}

Path::Path()
{
	initPath();
}


Path::~Path()
{
}

void Path::initPath()
{
	path.clear();
	path.resize(numSteps);
	updateLocations();
	updateCurvature();
	updateArcLen();
}

void Path::updateLocations()
{
	float step = 1. / numSteps;
	for (int i = 0; i < numSteps; i++) {
		path[i].location=posAtParam((float)i*step);
	}
}

void Path::updateCurvature()
{
	float step = 1. / numSteps;
	for (int i = 0; i < numSteps; i++) {
		path[i].curvature=curvAtParam((float)i*step);
	}
}

void Path::updateArcLen()
{
	float step = 1. / numSteps;
	path[0].arclen = 0;
	for (int i = 1; i < numSteps; i++) {
		path[i].arclen = path[i - 1].arclen + path[i - 1].location.dist(path[i].location);
	}
}
