#pragma once
#include <vector>

struct Vect
{
	float x = 0;
	float y = 0;

	float operator*(const Vect &b) const{
		return x*b.x+ y*b.y;
	}

	Vect operator*(const float &b) const {
		Vect out;
		out.x *= b;
		out.y *= b;
		return out;
	}

	Vect operator-(const Vect &b) const {
		Vect out;
		out.x -= b.x;
		out.y -= b.y;
		return out;
	}

	float dot(const Vect &b) const{
		return x*b.y - y*b.x;
	}

	float dist(const Vect &b) const {
		return sqrtf((x - b.x)*(x - b.x) + (y - b.y)*(y - b.y));
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

	void initPath();

	void updateLocations();
	void updateCurvature();
	void updateArcLen();

	Vect posAtParam(float param) { return Vect(); };
	Vect velAtParam(float param) { return Vect(); };
	Vect accAtParam(float param) { return Vect(); };
	float curvAtParam(float param) { return float(); };

	Path(int len);
	Path();
	~Path();
};

