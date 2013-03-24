#include <iostream>
#include <stdio.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/file_io.h>

// Segmentation includes...

#include "Common.h"
#include "Graph.h"
#include "IO.h"
#include "OverSegmentation.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>

#include <map>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image, sensor_msgs::PointCloud2> KinectSyncPolicy;

const std::string filename = "/media/Data/RGBD-Datasets/Freiburg/rgbd_dataset_freiburg3_cabinet-2hz-with-pointclouds.bag";
const std::string colorTopic = "/camera/rgb/image_color";
const std::string depthTopic = "/camera/depth/image";
const std::string pointTopic = "/camera/rgb/points";

// Global superpixel lookup by u,v index
Graph graph;
cv::Mat overSegmented;
Eigen::MatrixXf spLookup;
std::map<SuperPixelID, int> SPtoS;
std::vector<SuperPixelID> StoSP[4];
//std::map<int, SuperPixelID> StoSP;

// Currently selected S
int currentS = 0;
// Colors
const cv::Vec3b segColors[4] = {cv::Vec3b(255,0,0),cv::Vec3b(0,255,0),cv::Vec3b(0,0,255),cv::Vec3b(0,255,255)};


/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const> &msg)
  {
    signalMessage(msg);
  }
};

void segment(cv::Mat disparities, cv::Mat colors, pcl::PointCloud<pcl::PointXYZRGB> pCloud, cv::Mat cd);
void createLookup(const Graph& graph, Eigen::MatrixXf& lookup);
void repaintSuperPixel(const Graph& graph, cv::Mat& segmentedImage, SuperPixelID id, int segID);
void onMouse(int event, int x, int y, int flags, void* param);
void computeSuperPlanes();



void callback(const sensor_msgs::ImageConstPtr color, const sensor_msgs::ImageConstPtr depth, const sensor_msgs::PointCloud2ConstPtr points)
{
	cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(color, "bgr8");
	std::cout << "Callback called.\n";
	cv::imshow("image", imgPtr->image);

}

int main(int argc, char** argv)
{
	cv::Mat disparities, colors, depth3;
	disparities = *(disparityImage(cv::imread("/home/arprice/workspace/eclipseMCMC/depth1.png")));
	colors = cv::imread("/home/arprice/workspace/eclipseMCMC/color1.png");
	depth3 = cv::imread("/home/arprice/workspace/eclipseMCMC/depth1.png");

	ros::Time::init();
	//ros::init(argc, argv, "simple_segment");
	cv::imshow("c", colors);
	cv::imshow("d", disparities);
	//cv::waitKey(0);
	segment(disparities, colors, *(new pcl::PointCloud<pcl::PointXYZRGB>), depth3);
	return 0;


	// Open Bagfile
	rosbag::Bag bag;
	bag.open(filename, rosbag::bagmode::Read);

	// Select topics to load
	std::vector<std::string> topics;
	topics.push_back(colorTopic);
	topics.push_back(depthTopic);
	topics.push_back(pointTopic);
	rosbag::View view(bag, rosbag::TopicQuery(topics));


	BagSubscriber<sensor_msgs::Image> visualSub;
	BagSubscriber<sensor_msgs::Image> depthSub;
	BagSubscriber<sensor_msgs::PointCloud2> cloudSub;
	//message_filters::Synchronizer<KinectSyncPolicy> sync(KinectSyncPolicy(1), visualSub, depthSub, cloudSub);
	//sync.registerCallback(boost::bind(&callback, _1, _2, _3));

	message_filters::TimeSynchronizer<sensor_msgs::Image,sensor_msgs::Image, sensor_msgs::PointCloud2> tSync(visualSub, depthSub, cloudSub, 25);
	tSync.registerCallback(boost::bind(&callback, _1, _2, _3));
	tSync.init();
	tSync.setName("mySync");


	pcl::PointCloud<pcl::PointXYZRGB> pCloud;
	int state = 0;
	int counter = 0;

	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		if (m.getTopic() == colorTopic && (state & 0x01) != 0x01)
		{
			sensor_msgs::Image::ConstPtr ptr = m.instantiate<sensor_msgs::Image>();
			if (ptr != NULL)
			{
				state |= 0x01;
				std::cout << "New Color image. \n";
				visualSub.newMessage(ptr);
				cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(ptr, "bgr8");
				colors = imgPtr->image;
				cv::imshow("c", colors);
			}
		}
		else if (m.getTopic() == depthTopic && (state & 0x02) != 0x02)
		{
			sensor_msgs::Image::ConstPtr ptr = m.instantiate<sensor_msgs::Image>();
			if (ptr != NULL)
			{
				state |= 0x02;
				std::cout << "New Depth image. \n";
				depthSub.newMessage(ptr);
				cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(ptr, "mono8");
				depth3 = cv_bridge::toCvCopy(ptr, "bgr8")->image;
				disparities = imgPtr->image;
				cv::imshow("d", depth3);
			}
		}
		else if (m.getTopic() == pointTopic && (state & 0x04) != 0x04)
		{
			sensor_msgs::PointCloud2::ConstPtr ptr = m.instantiate<sensor_msgs::PointCloud2>();
			if (ptr != NULL)
			{
				state |= 0x04;
				std::cout << "New Cloud image. \n";
				cloudSub.newMessage(ptr);
				pcl::fromROSMsg(*ptr, pCloud);

			}
		}

		if (state == (1|2|4))
		{
			std::cout << "New full set. \n";
			counter++;

			if (counter >= 8)
			{
				counter = 0;
				cv::waitKey(0);
				segment(disparities, colors, pCloud, depth3);
			}

			state = 0;
		}
	}

	bag.close();


	return 0;
}

void segment(cv::Mat d, cv::Mat c, pcl::PointCloud<pcl::PointXYZRGB> pCloud, cv::Mat cd)
{
	srand(time(NULL));
	cv::Mat disparities, colors, filtd, filtc, filtcd;
	c.copyTo(colors);
	d.copyTo(disparities);

	OverSegmentationParameters params;
	IO::readSegmentationParams("overSegmentationParams.txt", params);
	params.print();

	size_t window = params.windowSize_;

	std::cout << "checkpoint 1.\n";
	for(int i = 0; i < 3; i++)
	{
		cv::bilateralFilter(colors, filtc, window, window*2, window/2);
		filtc.copyTo(colors);

		cv::bilateralFilter(disparities, filtd, window, window*2, window);
		filtd.copyTo(disparities);

		cv::bilateralFilter(cd, filtcd, window, window*2, window);
		filtcd.copyTo(cd);
	}
	int res = system("mkdir -p ./temp/superPixels/");
	cv::imwrite("./temp/superPixels/colorf.png", colors);
	cv::imwrite("./temp/superPixels/depthf.png", cd);

	// Segmentation call
	//Graph graph;
	gttic_(OverSegmentation);
	OverSegmentation::overSegment(disparities, colors, params, graph);
	gttoc_(OverSegmentation);
	tictoc_print_();

	// Get the color image
	//cv::Mat overSegmented;
	OverSegmentation::visualize(graph, overSegmented);
	createLookup(graph, spLookup);
	cv::imwrite("./temp/superPixels/overSegmented.png", overSegmented);

	std::map<SuperPixelID, SuperPixel*>::iterator iter;
	std::stringstream mapping;
	std::string idName;
	int count = 0;

	/*
	for (iter = graph.superPixels_.begin(); iter != graph.superPixels_.end(); ++iter)
	{
		count++;
		//mapping << ((long unsigned int)(iter->first));
		//mapping << ("->\n");
		//mapping << iter->second->A_->matrix();//((long unsigned int)(iter->second->id_));
		//mapping << ("\n");

		// Get ID
		idName = std::to_string((long unsigned int)(iter->first));

		// Get central pixel (middle?)
		int len = iter->second->A_->rows();
		int u = (int)(*(iter->second->A_))((int)(len/2),0);
		int v = (int)(*(iter->second->A_))((int)(len/2),1);

		// Write id @ pixel location
		cv::putText(overSegmented, idName, cv::Point(u,v),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,255,255), 1, CV_AA);

	}
	std::cout << "SuperPixels: " << count << std::endl;
	*/

	cv::namedWindow("Result");
	cv::setMouseCallback("Result", onMouse, 0);
	cv::imshow("Result", overSegmented);

	cv::waitKey(0);

	//std::ofstream outfile;
	//outfile.open("./temp/superPixels/mapping.txt");
	//outfile << mapping.str();

	//FILE* pFile = fopen("mapping.txt", "w");
	//fprintf(pFile, "%s", mapping.str().c_str());

}

/* ********************************************************************************************** */
void createLookup(const Graph& graph, Eigen::MatrixXf& lookup)
{
	lookup = Eigen::MatrixXf::Zero(kHeight, kWidth);

	// Set the superpixel index for each pixel
	std::map <SuperPixelID, SuperPixel*>::const_iterator superPixelIt = graph.superPixels_.begin();
	for(; superPixelIt != graph.superPixels_.end(); superPixelIt++)
	{
		// Set the colors for the pixels
		for(size_t pixelIdx = 0; pixelIdx < superPixelIt->second->A_->rows(); pixelIdx++)
		{
			size_t u = (*superPixelIt->second->A_)(pixelIdx, 0), v = (*superPixelIt->second->A_)(pixelIdx, 1);
			lookup(v,u) = superPixelIt->first;
		}
	}
}

void repaintSuperPixel(const Graph& graph, cv::Mat& segmentedImage, SuperPixelID id, int segID)
{
	// Find the superpixel index for each pixel
	std::map <SuperPixelID, SuperPixel*>::const_iterator superPixelIt = graph.superPixels_.find(id);//.begin();

	// Set the colors for the pixels
	for(size_t pixelIdx = 0; pixelIdx < superPixelIt->second->A_->rows(); pixelIdx++)
	{
		size_t u = (*superPixelIt->second->A_)(pixelIdx, 0), v = (*superPixelIt->second->A_)(pixelIdx, 1);
		segmentedImage.at<cv::Vec3b>(v,u) = segColors[segID];
	}

}

void onMouse(int event, int x, int y, int flags, void* param)
{

    if (event == CV_EVENT_LBUTTONDOWN)
    {
    	// Add current superpixel to current S
    	SuperPixelID sID = (SuperPixelID)spLookup(y,x);

    	std::cout << "x=" << x << ", " <<
    	        	 "y=" << y << ", " <<
    	        	 "s=" << sID << std::endl;
    	//StoSP.insert(std::pair<SuperPixelID, int>(sID, currentS));
    	StoSP[currentS].push_back(sID);
    	SPtoS.insert(std::pair<int, SuperPixelID>(currentS, sID));
    	// Recolor image
    	repaintSuperPixel(graph, overSegmented, sID, currentS);
    	cv::imshow("Result", overSegmented);
    	
    	std::map<SuperPixelID, SuperPixel*>::const_iterator superPixelIt = graph.superPixels_.find(sID);
    	Plane p(*(superPixelIt->second->A_),*(superPixelIt->second->b_));
		gtsam::Vector local = disparityToExplicit(p.density_.mean());
		gtsam::Vector original = (superPixelIt->second->plane_->density_.mean());
		std::cout << "Local Plane Parameters: " << local.matrix().transpose() << std::endl;
		std::cout << "Plane Parameters Disparity: " << original.matrix().transpose() << std::endl;
    }
    else if (event == CV_EVENT_RBUTTONDOWN)
    {
    	// Prompt for new S
    	//std::string newS;
    	//std::cout << "Select new S value: 0=A; 1=B; 2=C; 3=D : \n";
    	//std::cin >> newS;
    	//currentS = atoi(newS.c_str());
    	currentS++;
    	currentS = currentS % 4;
    	std::cout << "S index: " << currentS << std::endl;
    }
    else if (event == CV_EVENT_MBUTTONDOWN)
    {
    	// go!
    	cv::imwrite("./temp/superPixels/overSegmented.png", overSegmented);
    	computeSuperPlanes();
    }
}

void computeSuperPlanes()
{
	for (int i = 0; i < 4; i++)
	{
		std::cout << "S_" << (i) << ":\n";
		//SuperPixel ssp;
		//ssp.A_ = new Matrix();
		//ssp.b_ = new Matrix();
		Eigen::MatrixXd A;// = new Eigen::MatrixXd();
		Eigen::MatrixXd b;// = new Eigen::MatrixXd();
		Eigen::MatrixXd tempA;
		Eigen::MatrixXd tempb;
		A.resize(0,3);
		b.resize(0,1);

		for (int j = 0; j < StoSP[i].size(); j++)
		{
			//std::cout << "\t" << (j) << ":" << StoSP[i][j] << "\n";

			std::map <SuperPixelID, SuperPixel*>::const_iterator superPixelIt = graph.superPixels_.find(StoSP[i][j]);

			gtsam::Vector local = disparityToExplicit(superPixelIt->second->plane_->density_.mean());
			std::cout << j << " Plane Parameters: " << local.matrix().transpose() << std::endl;

			tempA.resize(A.rows()+superPixelIt->second->A_->rows(),superPixelIt->second->A_->cols());
			//std::cout << A.rows() << "x" << A.cols() << "\n";
			//std::cout << superPixelIt->second->A_->rows() << "x" << superPixelIt->second->A_->cols() << "\n";
			//std::cout << tempA.rows() << "x" << tempA.cols() << "\n";
			tempA << A,*(superPixelIt->second->A_);
			//std::cout << "TempA: " << tempA.rows() << "x" << tempA.cols() << "\n";
			A = tempA;
			//std::cout << A.rows()+superPixelIt->second->A_->rows() << "," << tempA.rows() << "\n";
			//std::cout << "First!\n";

			tempb.resize(b.rows()+superPixelIt->second->b_->rows(),superPixelIt->second->b_->cols());
			//std::cout << b.rows() << "x" << b.cols() << "\n";
			//std::cout << superPixelIt->second->b_->rows() << "x" << superPixelIt->second->b_->cols() << "\n";
			//std::cout << tempb.rows() << "x" << tempb.cols() << "\n";
			tempb << b,*(superPixelIt->second->b_);
			b = tempb;
			//std::cout << "Second!\n";
		}

		if (A.rows() > 0)
		{
			Plane p(A,b);
			gtsam::Vector local = disparityToExplicit(p.density_.mean());
			std::cout << "Plane Parameters: " << local.matrix() << std::endl;
			std::cout << "Error: " << p.error_ << "\tNormalized: " << p.error_/A.rows() << std::endl;
		}

	}
}
