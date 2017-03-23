#pragma once
#include "Path.h"
class Curve :
	public Path
{
public:

	struct QuinticBez
	{
		std::vector<Vect> start;
		std::vector<Vect> end;
		std::vector<Vect> initTangent;
		std::vector<Vect> finTangent;
	};

	void posAtParam(float param);
	void velAtParam(float param);
	void accAtParam(float param);
	void curvAtParam(float param);

	Curve();
	~Curve();
};

