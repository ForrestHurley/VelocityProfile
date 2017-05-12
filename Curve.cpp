#include "Curve.h"

Vect Curve::posAtParam(float param)
{
	int segment;
	float segParam;
	getSegmentParam(segment, segParam, param);

	Vect out;
	out = Bezier(route.parameters[segment], segParam);
	return out;
}

Vect Curve::velAtParam(float param)
{
	int segment;
	float segParam;
	getSegmentParam(segment, segParam, param);

	Vect out;
	out = Bezier(DiffBezier(route.parameters[segment]), segParam);
	return out;
}

Vect Curve::accAtParam(float param)
{
	int segment;
	float segParam;
	getSegmentParam(segment, segParam, param);

	Vect out;
	out = Bezier(DiffBezier(DiffBezier(route.parameters[segment])), segParam);
	return out;
}

std::vector<int> Curve::PascalTriangle(int row) {
	row -= 1;
	int x = 1;
	std::vector<int> out;
	out.push_back(1);
	for (int i = 0; i < row; i++) {
		x *= (float)(row - i) / (float)(i + 1);
		out.push_back(x);
	}
	return out;
}

float Curve::curvAtParam(float param)
{
	float out;
	out = abs(accAtParam(param).cross(velAtParam(param)));
	float magnV = velAtParam(param).magnitude();
	out /= magnV*magnV*magnV;

	return out;
}

Curve::PointSet Curve::DiffBezier(PointSet points)
{
	PointSet out;
	for (int i = 0; i < points.points.size() - 1; i++) {
		out.points.push_back((points.points[i + 1] - points.points[i]) * (float)points.points.size());
	}

	return out;
}

Vect Curve::Bezier(PointSet points, float param)
{
	Vect out;
	std::vector<int> pascal = PascalTriangle(points.points.size());
	float marap = 1. - param;
	for (int i = 0; i < points.points.size(); i++) {
		out.x += (float)pascal[i] * points.points[i].x*powf(param, i)*powf(marap, points.points.size() - i - 1);
		out.y += (float)pascal[i] * points.points[i].y*powf(param, i)*powf(marap, points.points.size() - i - 1);
	}
	return out;
}

void Curve::getSegmentParam(int & segment, float & param, float paramIn)
{
	segment = paramIn * route.parameters.size();
	param = paramIn - ((float)segment / (float)route.parameters.size());
}

Curve::Curve()
{
}

Curve::Curve(PointSet initPointsA, PointSet initPointsB)
{
	addSegment(initPointsA);
	addSegment(initPointsB);
}


Curve::~Curve()
{
}
