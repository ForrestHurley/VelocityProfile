#include <iostream>
#include "Curve.h"
#include "OCVPath.h"
#include "VectorOptim.h"

int main()
{
	Curve::PointSet testPointsA, testPointsB;
	testPointsA.points.push_back(Vect(200,50));
	testPointsA.points.push_back(Vect(400, 300));
	testPointsA.points.push_back(Vect(10, 500));
	/*testPointsA.points.push_back(Vect(150, 300));
	testPointsA.points.push_back(Vect(200, 500));*/
	Curve test;

	test.addSegment(testPointsA);
	//test.addSegment(testPointsB);

	test.updateLocations();
	test.updateCurvature();
	VectorOptim profiler;
	profiler.maxAcceleration = 3;
	profiler.maxJerk = 1;
	profiler.maxVelocity = 30;
	profiler.generateProfile(&test);

	OCVPath::ChartVelocity(test);
	OCVPath::ChartAcceleration(test);
	OCVPath::ChartJerk(test);
	OCVPath::DrawPath(test);
	OCVPath::ChartCurvature(test);

	return 0;
}