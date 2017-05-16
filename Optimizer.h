#pragma once
#include "Path.h"
#include <iostream>
class Optimizer
{
public:

	struct bounds {
		float min;
		float max;

		//Performs an intersection on two bounds
		void combine(const bounds &b) {
			if (isnan(max) || isnan(b.max))
				max = NAN;
			else max = std::fminf(max, b.max);
			if (isnan(min) || isnan(b.min))
				min = NAN;
			else min = std::fmaxf(min, b.min);
		}

		void setBound(float minIn, float maxIn) {
			min = minIn;
			max = maxIn;
		}

		//If there are no real numbers greater than min and less than max, returns true
		bool null() {
			if (min - max > 0.001 || isnan(min) || isnan(max)) return true;
			return false;
		}

		bounds() {
			min = -INFINITY;
			max = INFINITY;
		}

		bounds(float minIn, float maxIn) {
			min = minIn;
			max = maxIn;
		}
	};

	float maximumVelocity = 10;
	float minimumVelocity = 0;

	float initialVelocity = 0;
	float finalVelocity = 0;

	bool verbose = false;

	//Generates the velocity profile for a path
	void generateProfile(Path *in);

	Optimizer();
	~Optimizer();

private:

	//The main recursion algorithm. It runs back and forth along the path, reducing velocities until an optimum solution is found
	void recurOptim(Path *in, int iter, int forward);

	//Initialize the velocities along the path
	void initMaxVel(Path *in);

	void updateVelocity(Path *in, int iter, float maxVelocity);
	virtual void updateParameters(Path *in, int iter, int forward) {};

	//For creating constraints on the maximum and minimum velocity based on the velocities at previous points on the path
	virtual bounds backCheck(Path *in, int iter, int forward) { return bounds(); };

	//For creating purely static constraints which are only dependent on the current state of the path and not future or past states. For example, an absolute maximum velocity
	virtual bounds staticCheck(Path *in, int iter, int forward) { return bounds(); };

	virtual bounds foreCheck(Path *in, int iter, int forward) { return bounds(); };

};

