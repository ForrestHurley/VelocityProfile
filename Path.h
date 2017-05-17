#pragma once
#include <vector>

struct Vect
{
	float x = 0;
	float y = 0;

	float operator*(const Vect &b) const{
		return x*b.x+ y*b.y;
	}

	Vect(float a, float b) {
		x = a;
		y = b;
	}

	Vect(){}

	Vect operator*(const float &b) const {
		Vect out;
		out.x = x*b;
		out.y = y*b;
		return out;
	}
	
	Vect operator/(const float &b) const {
		Vect out;
		out.x = x/b;
		out.y = y/b;
		return out;
	}

	Vect operator-(const Vect &b) const {
		Vect out;
		out.x = x-b.x;
		out.y = y-b.y;
		return out;
	}

	Vect operator+(const Vect &b) const {
		Vect out;
		out.x = x + b.x;
		out.y = y + b.y;
		return out;
	}

	Vect normalize() const {
		float magn = magnitude();
		if (magn != 0)
			return *this / magnitude();
		else
			return Vect(0,0);
	}

	Vect rotNinty() const {
		Vect out;
		out.x = y;
		out.y = -x;
		return out;
	}

	float dot(const Vect &b) const{
		return x*b.x + y*b.y;
	}

	float cross(const Vect &b) const {
		return x*b.y - y*b.x;
	}

	float dist(const Vect &b) const {
		return sqrtf((x - b.x)*(x - b.x) + (y - b.y)*(y - b.y));
	}

	float magnitude() const {
		return sqrtf(x*x + y*y);
	}

	float sine(const Vect &b) const {
		return normalize().cross(b.normalize());
	}
};

class Path
{
public:

	int numSteps = 500;

	struct Segment
	{
		Vect location;
		Vect velocity;
		Vect acceleration;
		float proportion;
		float arclen;
		float time;
		float curvature;
	};

	std::vector<Segment> path;

	void initPath();

	void updateLocations();
	void updateParameters();
	void initVelocity();
	void updateCurvature();
	void updateArcLen();
	void updateAcceleration();
	void updateTime();

	virtual Vect posAtParam(float param) { return Vect(); };
	virtual Vect velAtParam(float param) { return Vect(); };
	virtual Vect accAtParam(float param) { return Vect(); };
	virtual float curvAtParam(float param) { return float(); };

	Path(int len);
	Path();
	~Path();
};

