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
			max = std::fminf(max, b.max);
			min = std::fmaxf(min, b.min);
		}

		void setBound(float minIn, float maxIn) {
			min = minIn;
			max = maxIn;
		}

		//If there are no real numbers greater than min and less than max, returns true
		bool null() {
			if (max < min) return true;
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

	//For creating constraints on the maximum and minimum velocity based on future positions on the path. It does not have access to future velocities
	virtual bounds foreCheck(Path *in, int iter, int forward) { return bounds(); };

	//For creating purely static constraints which are only dependent on the current state of the path and not future or past states. For example, an absolute maximum velocity
	virtual bounds staticCheck(Path *in, int iter, int forward) { return bounds(); };

};

