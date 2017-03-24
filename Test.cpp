#include <iostream>
#include "Curve.h"
#include "OCVPath.h"

int main()
{
	Curve test;
	test.PascalTriangle(5);
	
	OCVPath::DrawPath(test);

	return 0;
}