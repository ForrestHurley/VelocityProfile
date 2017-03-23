#pragma once
#include <vector>

struct Vect
{
	float x;
	float y;

	Vect operator*(const Vect &b) const{
		Vect out;
		out.x = x*b.x;
		out.y = y*b.y;
		return out;
	}

	float operator$(const Vect &b) const{
		return x*b.y - y*b.x;
	}
};

class Path
{
public:

	int numSteps = 30;

	struct Segment
	{
		Vect location;
		float proportion;
		float arclen;
		float time;
		float curvature;
	};

	std::vector<Segment> path;

	void updateLocations();
	void updateCurvature();
	void updateArcLen();

	void posAtParam(float param);
	void velAtParam(float param);
	void accAtParam(float param);
	void curvAtParam(float param);

	Path();
	~Path();
};

