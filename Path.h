#pragma once
#include <vector>

struct Vect
{
	float x;
	float y;
};

Vect operator*(const Vect &a, const Vect &b) {
	Vect out;
	out.x = a.x*b.x;
	out.y = a.y*b.y;
	return out;
}

float operator$(const Vect &a, const Vect &b) {
	return a.x*b.y - a.y*b.x;
}

class Path
{
public:
	struct Segment
	{
		Vect location;
		float proportion;
		float arclen;
		float time;
		float curvature;
	};

	std::vector<Segment> path;

	void updatePath();

	Path();
	~Path();
};

