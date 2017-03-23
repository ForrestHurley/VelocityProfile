#pragma once
#include "Path.h"
#include <opencv2\core.hpp>

class OCVPath
{
public:

	float lineWidth = 1;
	cv::Scalar color = cv::Scalar(110, 220, 0);

	void DrawPath(Path in);

	OCVPath();
	~OCVPath();
};

