#include "OCVPath.h"
#include <vector>
#include <iostream>

using namespace cv;

cv::Point OCVPath::vectToPoint(Vect in)
{
	return cv::Point(in.x,in.y);
}

void OCVPath::DrawPath(Path in)
{
	Mat image = Mat::zeros(500,500,CV_8UC3);

	for (int i = 0; i < in.path.size()-1; i++) {
		line(image, vectToPoint(in.path[i].location), vectToPoint(in.path[i + 1].location), Scalar(110, 220, 0), 2, 8);
	}
	
	//image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file

	namedWindow("Display window", WINDOW_AUTOSIZE);// Create a window for display.
	imshow("Display window", image);                   // Show our image inside it.

	waitKey(0);
}

void OCVPath::ChartCurvature(Path in)
{
	std::vector<float> toChart;
	for (int i = 0; i < in.path.size(); i++){
		toChart.push_back(in.path[i].curvature);
	}
	Chart(toChart);
}

void OCVPath::Chart(std::vector<float> in)
{
	int width = 500;
	int height = 500;
	Mat image = Mat::zeros(width, height, CV_8UC3);

	float maxVal = 0;

	for (int i = 0; i < in.size(); i++) {
		if (in[i] > maxVal) maxVal = in[i];
	}

	maxVal = 1. / maxVal;

	for (int i = 0; i < in.size() - 1; i++) {
		line(image, cv::Point((i * (float)width / (in.size())), height - (float)height*maxVal*in[i]), cv::Point(((i + 1) * (float)width / (in.size())), height - (float)height*maxVal*in[i + 1]), Scalar(110, 220, 0), 2, 8);
	}

	//image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file

	namedWindow("Display window", WINDOW_AUTOSIZE);// Create a window for display.
	imshow("Display window", image);                   // Show our image inside it.

	waitKey(0);
}

OCVPath::OCVPath()
{
}


OCVPath::~OCVPath()
{
}
