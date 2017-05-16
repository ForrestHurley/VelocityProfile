#include <iostream>
#include "Curve.h"
#include "OCVPath.h"
#include "VectorOptim.h"
#include "StandOptim.h"

int main()
{
	Curve::PointSet testPointsA, testPointsB;
	testPointsA.points.push_back(Vect(200,50));
	testPointsA.points.push_back(Vect(200, 300));
	//testPointsA.points.push_back(Vect(250, 50));
	//testPointsA.points.push_back(Vect(150, 300));
	//testPointsA.points.push_back(Vect(200, 459));
	Curve test;
	test.numSteps = 150;

	test.addSegment(testPointsA);
	//test.addSegment(testPointsB);

	test.initPath();
	StandOptim profiler;
	profiler.verbose = true;
	profiler.maximumAcceleration = 10;
	profiler.maximumJerk = 0.05;
	profiler.maximumVelocity = 20;
	profiler.generateProfile(&test);
	test.updateAcceleration();

	OCVPath::ChartVelocity(test);
	OCVPath::ChartAcceleration(test);
	OCVPath::ChartJerk(test);
	OCVPath::DrawPath(test);
	OCVPath::ChartCurvature(test);

	return 0;
}