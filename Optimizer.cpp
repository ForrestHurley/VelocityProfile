#include "Optimizer.h"
#include <iostream>


void Optimizer::generateProfile(Path * in)
{
	in->initPath();
	initMaxVel(in);
	in->path[0].velocity = in->path[0].velocity.normalize() * initialVelocity;
	in->path[in->path.size()-1].velocity = in->path[in->path.size() - 1].velocity.normalize() * finalVelocity;
	recurOptim(in, 1, 1);
}

Optimizer::Optimizer()
{
}


Optimizer::~Optimizer()
{
}

void Optimizer::recurOptim(Path * in, int iter, int forward)
{

	bounds lastBounds;
	bounds staticBounds;
	bounds nextBounds;
	bounds nonVelBounds;
	bounds finalBounds;

	while (true) {

		//Generate the maximum and minimum velocities at a point given knowledge of previous and future state respectively
		lastBounds = backCheck(in, iter, forward);
		staticBounds = staticCheck(in, iter, forward);
		nextBounds = foreCheck(in, iter, forward);

		//Perform a intersection operation on the bounds to get the range of speeds which satisfy both
		nonVelBounds = staticBounds;
		nonVelBounds.combine(nextBounds);

		if (nonVelBounds.null()) {
			std::cout << "No possible velocity profile. There may be an error in the static and forward looking kinematic constraints" << std::endl;
			break;
		}

		finalBounds = nonVelBounds;
		finalBounds.combine(lastBounds);
		
		//std::cout << "Last Bounds: " << lastBounds.max << " Next Bounds: " << nextBounds.max << " Static Bounds: " << staticBounds.max << std::endl;

		//If there are no possible velocities, recurse and flip directions
		if (finalBounds.null()) {
			std::cout << "Final Bounds Null, Minimum:" << finalBounds.min << " Maximum: " << finalBounds.max << " Iteration: " << iter << std::endl;
			std::cout << "lastBounds Minimum: " << lastBounds.min << " Maximum: " << lastBounds.max << std::endl;

			Vect prevVel = in->path[iter].velocity;
			updateVelocity(in, iter, nonVelBounds.max);
			updateParameters(in, iter, forward);

			recurOptim(in, iter - forward, -forward);
			
			in->path[iter].velocity = prevVel;
			updateParameters(in, iter, forward);

		}
		else if (finalBounds.min > in->path[iter].velocity.magnitude()) {
			std::cout << "Velocity below bounds, Iteration: " << iter << std::endl;
			recurOptim(in, iter - forward, -forward);
		}
		else {

			//If this recursion has succeeded in connecting back to previous values, move back up and continue the previous recursion
			if (finalBounds.max > in->path[iter].velocity.magnitude()) {
				std::cout << "Reached solution, regressing. Final Bounds: " << finalBounds.max << " Previous velocity: " << in->path[iter].velocity.magnitude() << " Iteration: " << iter << std::endl;
				
				//Update the path velocity at the current iteration
				//updateVelocity(in, iter, finalBounds.max);
				//updateParameters(in, iter, forward);
				
				break;
			}

			//Update the path velocity at the current iteration
			updateVelocity(in, iter, finalBounds.max);
			updateParameters(in, iter, forward);

			//Iterate the current point either forward or backward dependng on whether the value of forward is -1 or 1
			iter += forward;

			//If we have reached the end of the path, no more improvements can be made; go back to the previous recursion
			if ((iter < 0) || (iter > in->path.size() - 1)) {
				std::cout << "Reached end of path, Iteration: " << iter << std::endl;
				break;
			}

		}

	}

}

void Optimizer::initMaxVel(Path * in)
{
	//Set the velocity at each point on the path to be higher than the maximum velocity. This guarentees that the top level of the recursion runs all of the way through the path 
	for (int i = 0; i < in->path.size(); i++) {
		in->path[i].velocity = in->path[i].velocity.normalize()*(maximumVelocity + 1.);
	}
}

void Optimizer::updateVelocity(Path * in, int iter, float maxVelocityIn)
{
	//std::cout << maxVelocityIn << std::endl;
	float newVel = fminf(maxVelocityIn,in->path[iter].velocity.magnitude());
	in->path[iter].velocity = in->path[iter].velocity.normalize() * newVel;
}
