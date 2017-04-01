#pragma once
#include "Optimizer.h"
#include "Path.h"

class VectorOptim :
	public Optimizer
{
public:

	struct bounds {
		float min;
		float max;

		bounds combine(const bounds &b) const {
			bounds out;
			out.max = std::fminf(max, b.max);
			out.min = std::fminf(min, b.min);
			return out;
		}

		void setBound(float minIn, float maxIn) {
			min = minIn;
			max = maxIn;
		}

		bool null() {
			if (max < min) return true;
			return false;
		}

		bounds() {}

		bounds(float minIn, float maxIn) {
			min = minIn;
			max = maxIn;
		}
	};

	float maximumVelocity;
	float maximumAcceleration;
	float maximumJerk;

	void generateProfile(Path *in);

	VectorOptim();
	~VectorOptim();
private:
	void initMaxVel(Path *in);
	void recurOptim(Path *in, int iter, int forward, bounds(*backCheck)(Path *in, int iter, bool forward), bounds(*foreCheck)(Path *in, int iter, bool forward));
	
	bounds maxJerkKin(Path *in, int iter, int forward);
	bounds maxAccelerationKin(Path *in, int iter, int forward);
	bounds maxVelocityKin(Path *in, int iter, int forward);
	bounds kinematicBoundsKin(Path *in, int iter, int forward);

	bounds maxJerk(Path *in, int iter, int forward);
	bounds maxAcceleration(Path *in, int iter, int forward);
	bounds kinematicBounds(Path *in, int iter, int forward);

	float estimVelTime(Path *in, int iter, int forward);
	float getMaxAccMagn(Path *in, int iter, int forward);
	float getMaxJerkMagn(Path *in, int iter, int forward);

	float getArcLength(Path *in, int iter, int forward);
};

