#include <iostream>
#include "Curve.h"
#include "OCVPath.h"
#include "VectorOptim.h"
#include "StandOptim.h"

int main()
{
	Curve::PointSet testPointsA, testPointsB;
	testPointsA.points.push_back(Vect(200,50));
	testPointsA.points.push_back(Vect(50, 300));
	testPointsA.points.push_back(Vect(250, 100));
	testPointsA.points.push_back(Vect(25, 300));
	testPointsA.points.push_back(Vect(400, 300));
	testPointsA.points.push_back(Vect(50, 100));
	Curve test;
	test.numSteps = 150;

	test.addSegment(testPointsA);
	//test.addSegment(testPointsB);

	test.initPath();
	StandOptim profiler;
	profiler.verbose = false;
	profiler.maximumAcceleration = 10;
	profiler.maximumJerk = 0.05;
	profiler.maximumVelocity = 20;
	profiler.generateProfile(&test);
	test.updateAcceleration();

	OCVPath::ChartPositionVsTime(test);
	OCVPath::ChartVelocity(test);
	OCVPath::ChartAcceleration(test);
	OCVPath::ChartJerk(test);
	OCVPath::DrawPath(test);
	OCVPath::ChartCurvature(test);

	std::cout << "Time,Distance Traveled,Velocity,Acceleration,Curvature,Position X,Position Y" << std::endl;
	for (int i = 0; i < test.path.size(); i++) {
		std::cout << test.path[i].time << "," << test.path[i].arclen << "," << test.path[i].velocity.magnitude() << "," << test.path[i].acceleration.magnitude() << "," << test.path[i].curvature << "," << test.path[i].location.x << "," << test.path[i].location.y << std::endl;
	}

	return 0;
}