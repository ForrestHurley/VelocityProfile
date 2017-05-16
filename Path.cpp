#include "Path.h"
#include <iostream>

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
	updateParameters();
	initVelocity();
	updateCurvature();
	updateArcLen();
	updateAcceleration();
}

void Path::updateLocations()
{
	float step = 1. / numSteps;
	for (int i = 0; i < numSteps; i++) {
		path[i].location=posAtParam((float)i*step);
	}
}

void Path::updateParameters()
{
	float step = 1. / numSteps;
	for (int i = 0; i < numSteps; i++) {
		path[i].proportion = (float)i*step;
	}
}

void Path::initVelocity()
{
	float step = 1. / numSteps;
	for (int i = 0; i < numSteps; i++) {
		path[i].velocity = velAtParam((float)i*step).normalize();
	}
}

void Path::updateCurvature()
{
	float step = 1. / numSteps;
	for (int i = 0; i < numSteps; i++) {
		path[i].curvature=curvAtParam((float)i*step);
		//std::cout << path[i].curvature << std::endl;
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

void Path::updateAcceleration()
{
	path[0].acceleration = Vect(0,0);
	for (int i = 1; i < numSteps; i++) {
		path[i].acceleration = path[i].velocity - path[i - 1].velocity;
	}
}
