#include <iostream>
#include "Curve.h"
#include "OCVPath.h"
#include "VectorOptim.h"
#include "StandOptim.h"

int main()
{
	Curve::PointSet testPointsA, testPointsB;
	testPointsA.points.push_back(Vect(200,50));
	testPointsA.points.push_back(Vect(400, 300));
	testPointsA.points.push_back(Vect(10, 500));
	testPointsA.points.push_back(Vect(150, 300));
	testPointsA.points.push_back(Vect(200, 500));
	Curve test;

	test.addSegment(testPointsA);
	//test.addSegment(testPointsB);

	test.initPath();
	StandOptim profiler;
	profiler.maximumAcceleration = 3;
	//profiler.maximumJerk = 1;
	profiler.maximumVelocity = 20;
	profiler.generateProfile(&test);

	OCVPath::ChartVelocity(test);
	//OCVPath::ChartAcceleration(test);
	//OCVPath::ChartJerk(test);
	OCVPath::DrawPath(test);
	OCVPath::ChartCurvature(test);

	return 0;
}