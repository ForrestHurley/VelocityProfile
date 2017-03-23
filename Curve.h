#pragma once
#include "Path.h"
class Curve :
	public Path
{
public:

	struct QuinticBez
	{
		std::vector<Point> start;
		std::vector<Point> end;
		std::vector<Point> initTangent;
		std::vector<Point> finTangent;
	};

	Curve();
	~Curve();
};

