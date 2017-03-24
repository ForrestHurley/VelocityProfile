#pragma once
#include "Path.h"
#include "iostream"
class Curve :
	public Path
{
public:

	struct PointSet
	{
		std::vector<Vect> points;
	};

	struct Spline
	{
		std::vector<PointSet> parameters;
	};

	Spline route;

	Vect posAtParam(float param);
	Vect velAtParam(float param);
	Vect accAtParam(float param);
	float curvAtParam(float param);

	PointSet DiffBezier(PointSet points);

	Vect Bezier(PointSet points, float param);

	std::vector<int> PascalTriangle(int row);

	void getSegmentParam(int& segment, float& param, float paramIn);

	Curve();
	~Curve();
};

