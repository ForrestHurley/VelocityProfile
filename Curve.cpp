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
	int segment;
	float segParam;
	getSegmentParam(segment, segParam, param);
	return 0.0;
}

Curve::PointSet Curve::DiffBezier(PointSet points)
{
	PointSet out;
	for (int i = 0; i < points.points.size(); i++) {

	}

	return out;
}

Vect Curve::Bezier(PointSet points, float param)
{
	Vect out;
	float marap = 1. - param;
	for (int i = 0; i < points.points.size(); i++) {
		out.x += points.points[i].x*powf(param, i)*powf(marap, points.points.size() - i);
		out.y += points.points[i].y*powf(param, i)*powf(marap, points.points.size() - i);
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


Curve::~Curve()
{
}
