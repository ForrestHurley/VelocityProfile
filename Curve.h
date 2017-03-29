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

		void setPointsByDerivatives(std::vector<Vect> derivativesStart, std::vector<Vect> derivativesEnd) {
			std::vector<int> pascal = PascalTriangle(derivativesStart.size());
			for (int i = 0; i < derivativesStart.size(); i++) {

			}
		}
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

	void addSegment(PointSet pointsIn) {
		route.parameters.push_back(pointsIn);
	}

	PointSet DiffBezier(PointSet points);

	Vect Bezier(PointSet points, float param);

	static std::vector<int> PascalTriangle(int row);

	void getSegmentParam(int& segment, float& param, float paramIn);

	Curve();
	Curve(PointSet initPointsA, PointSet initPointsB);
	~Curve();
};

