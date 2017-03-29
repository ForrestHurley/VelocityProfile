#pragma once
#include "Path.h"
class Optimizer
{
public:

	virtual void generateProfile(Path *in) {};

	Optimizer();
	~Optimizer();
};

