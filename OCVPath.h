#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Path.h"

using namespace cv;

class OCVPath
{
public:

	float lineWidth = 1;
	cv::Scalar color = cv::Scalar(110, 220, 0);

	static void DrawPath(Path in);

	OCVPath();
	~OCVPath();
};

