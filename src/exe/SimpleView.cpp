#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

int main(int argc, char** argv)
{
	ros::Time::init();
	cv::namedWindow("kittens");
	cv::imshow("kittens",cv::imread("color.png"));
	cv::waitKey();
}
