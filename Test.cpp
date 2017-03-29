#include <iostream>
#include "Curve.h"
#include "OCVPath.h"

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

	test.updateLocations();
	test.updateCurvature();
	OCVPath::DrawPath(test);
	OCVPath::ChartCurvature(test);

	return 0;
}