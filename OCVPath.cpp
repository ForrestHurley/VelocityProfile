#include "OCVPath.h"
#include <vector>

using namespace cv;

void OCVPath::DrawPath(Path in)
{
	Mat image = Mat::zeros(500,500,CV_8UC3);
	
	//image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file
	line(image, Point(15, 20), Point(70, 50), Scalar(110, 220, 0), 2, 8);

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
