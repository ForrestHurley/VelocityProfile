#pragma once
#include "Optimizer.h"
#include "Path.h"

class VectorOptim :
	public Optimizer
{
public:

	float maximumAcceleration;
	float maximumJerk;

	float finalVelocity = 0;

	VectorOptim();
	~VectorOptim();

private:

	//Override virtual functions from optimizer to add kinematic functionality

	bounds backCheck(Path *in, int iter, int forward);
	bounds foreCheck(Path *in, int iter, int forward);
	bounds staticCheck(Path *in, int iter, int forward);
	

	//For the bounds checks based on previous states
	//Returns max velocity based on jerk from on previous velocity and acceleration and the change in path tangent
	bounds maxJerkKin(Path *in, int iter, int forward);
	//Returns max velocity based on acceleration from on previous velocity and the change in path tangent
	bounds maxAccelerationKin(Path *in, int iter, int forward);
	//Returns the intersection of acceleration and jerk based limits
	bounds kinematicBoundsKin(Path *in, int iter, int forward);


	//For the bounds checks based on future states
	//Returns the maximum velocity based on jerk from the future change in path tangent
	bounds maxJerk(Path *in, int iter, int forward);
	//Returns the maximum velocity based on acceleration from the future change in path tangent
	bounds maxAcceleration(Path *in, int iter, int forward);
	//Returns the intersection of acceleration and jerk based limits
	bounds kinematicBounds(Path *in, int iter, int forward);


	//For the static bounds
	//Preserves maximum velocity bound and allows the initial and final velocities to be set
	bounds maxVelocity(Path *in, int iter, int forward);

	//Estimates the delta time based on arc length and velocity
	float estimVelTime(Path *in, int iter, int forward);
	//Estimates the maximum acceleration vector magnitude based on delta time and maximum acceleration
	float getMaxAccMagn(Path *in, int iter, int forward);
	//Estimates the maximum jerk vector magnitude based on delta time and maximum jerk
	float getMaxJerkMagn(Path *in, int iter, int forward);
	//Gets delta arc length from the previously calculated path absolute arc lengths
	float getArcLength(Path *in, int iter, int forward);

	void updateParameters(Path *in, int iter, int forward);
	void updateAcceleration(Path *in, int iter,int forward);
};

