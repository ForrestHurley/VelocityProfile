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

	static cv::Point vectToPoint(Vect in);

	static void DrawPath(Path in);
	static void ChartCurvature(Path in);
	static void ChartVelocity(Path in);
	static void ChartAcceleration(Path in);
	static void ChartJerk(Path in);
	static void Chart(std::vector<float> in);
	static void Chart(std::vector<float> inX,std::vector<float> inY);

	OCVPath();
	~OCVPath();
};

