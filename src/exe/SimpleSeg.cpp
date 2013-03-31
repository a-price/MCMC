#define USE_BAGFILE

#include <iostream>
#include <stdio.h>

#include <ros/ros.h>
//#include <rosbag/bag.h>
//#include <rosbag/view.h>

//#include <boost/foreach.hpp>

#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/ros/conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
//#include <pcl/io/file_io.h>

// Segmentation includes...

#include "Common.h"
#include "Graph.h"
#include "IO.h"
#include "OverSegmentation.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>

#include <map>
// Visualization stuff

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image, sensor_msgs::PointCloud2> KinectSyncPolicy;

const std::string bagFilename = "/media/Data/RGBD-Datasets/Freiburg/rgbd_dataset_freiburg3_cabinet-2hz-with-pointclouds.bag";
const std::string paramFilename = "/home/arprice/fuerte_workspace/sandbox/MCMC/bin/overSegmentationParams.txt";
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
ros::Publisher superPixelPub;
ros::Publisher hyperPixelPub;

void kinectCallback(const sensor_msgs::ImageConstPtr color, const sensor_msgs::ImageConstPtr depth, const sensor_msgs::PointCloud2ConstPtr points);
void segment(cv::Mat disparities, cv::Mat colors, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);
void createLookup(const Graph& graph, Eigen::MatrixXf& lookup);
void repaintSuperPixel(const Graph& graph, cv::Mat& segmentedImage, SuperPixelID id, int segID);
void onMouse(int event, int x, int y, int flags, void* param);
void computeSuperPlanes();

Eigen::Quaterniond getRotation(Eigen::Vector3d axis)
{
	// Assume @ (1,0,0) originally
	if (axis.norm() - 1.0 > 0.001) {axis.normalize();}
	Eigen::Vector3d q = Eigen::Vector3d::UnitX().cross(axis);
	double angle = acos(q.dot(axis));
	double sinAngle = sin(angle/2.0);
	double cosAngle = cos(angle/2.0);


	Eigen::Quaterniond ret;
	ret.x() = q.x() * sinAngle;
	ret.y() = q.y() * sinAngle;
	ret.z() = q.z() * sinAngle;
	ret.w() = cosAngle;

	return ret;
}

int main(int argc, char** argv)
{
	Eigen::Quaterniond q = getRotation(Eigen::Vector3d::UnitY());
	std::cout << q.x() << "\t" << q.y() << "\t" << q.z() << "\t" << q.w() << std::endl;
	int r = system("pwd");
	ros::Time::init();
	cv::Mat disparities, colors, depth3;

#ifndef USE_BAGFILE

	disparities = *(disparityImage(cv::imread("/home/arprice/workspace/eclipseMCMC/depth1.png")));
	colors = cv::imread("/home/arprice/workspace/eclipseMCMC/color1.png");
	depth3 = cv::imread("/home/arprice/workspace/eclipseMCMC/depth1.png");


	//ros::init(argc, argv, "simple_segment");
	cv::imshow("c", colors);
	cv::imshow("d", disparities);
	//cv::waitKey(0);
	segment(disparities, colors, *(new pcl::PointCloud<pcl::PointXYZ>));
	return 0;

#else
	ros::init(argc, argv, "simple_seg");
	ros::NodeHandle nh_;

	superPixelPub = nh_.advertise<visualization_msgs::MarkerArray>( "superpixel_vectors", 0 );
	hyperPixelPub = nh_.advertise<visualization_msgs::MarkerArray>( "hyperpixel_vectors", 0 );

	message_filters::Subscriber<sensor_msgs::Image> color_sub_ (nh_, colorTopic, 8);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub_ (nh_, depthTopic, 8);
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_ (nh_, pointTopic, 8);
	message_filters::Synchronizer<KinectSyncPolicy> sync_(KinectSyncPolicy(8), color_sub_, depth_sub_, cloud_sub_);

	sync_.registerCallback(boost::bind(&kinectCallback, _1, _2, _3));
	ros::spin();
#endif

}

const int maxCallbacks = 1;
int numCallbacks = 0;
void kinectCallback(const sensor_msgs::ImageConstPtr color, const sensor_msgs::ImageConstPtr depth, const sensor_msgs::PointCloud2ConstPtr points)
{
	if (numCallbacks < maxCallbacks)
	{
		numCallbacks++;
	}
	else
	{
		return;
	}

	ROS_DEBUG("Got Callback.");
	cv_bridge::CvImagePtr cPtr = cv_bridge::toCvCopy(color, "bgr8");
	cv_bridge::CvImagePtr dPtr = cv_bridge::toCvCopy(depth, "mono8");
	pcl::PointCloud<pcl::PointXYZ> pCloud; // no color in this topic?
	pcl::fromROSMsg(*points, pCloud);
	ROS_DEBUG("Got Cloud.");

	// TODO:convert to disparity?
	segment(dPtr->image, cPtr->image, pcl::PointCloud<pcl::PointXYZ>::Ptr(&pCloud));
	//segment(*disparityImage(dPtr->image), cPtr->image, pcl::PointCloud<pcl::PointXYZ>::Ptr(&pCloud));

}

void segment(cv::Mat d, cv::Mat c, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud) //, cv::Mat cd)
{
	srand(time(NULL));
	cv::Mat disparities, colors, filtd, filtc, filtcd;
	c.copyTo(colors);
	d.copyTo(disparities);

	OverSegmentationParameters params;
	IO::readSegmentationParams(paramFilename, params);
	params.print();

	size_t window = params.windowSize_;

	std::cout << "checkpoint 1.\n";
	for(int i = 0; i < 3; i++)
	{
		cv::bilateralFilter(colors, filtc, window, window*2, window/2);
		filtc.copyTo(colors);

		cv::bilateralFilter(disparities, filtd, window, window*2, window);
		filtd.copyTo(disparities);
	}
	ROS_DEBUG("Got Segmentation.");
	int res = system("mkdir -p ./temp/superPixels/");
	cv::imwrite("./temp/superPixels/colorf.png", colors);

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
	//std::stringstream mapping;
	std::string idName;
	int count = 0;

	std::ofstream outfile;
	outfile.open("./temp/superPixels/vectors.txt");
	visualization_msgs::MarkerArray mArray;

	ROS_INFO("Got SPs.");
	for (iter = graph.superPixels_.begin(); iter != graph.superPixels_.end(); ++iter)
	{
		count++;
		Eigen::Vector4d plane = disparityToExplicit(iter->second->plane_->density_.mean());
		//plane.normalize();
		outfile << "0 0 0 " << plane[0] << " " << plane[1] << " " << plane[2] << " " << "\n";

		// Get ID
		idName = std::to_string((long unsigned int)(iter->first));

		ROS_DEBUG("Getting Indices.");
		// Get central pixel (middle?)
		int len = iter->second->A_->rows();
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
		for (int i = 0; i < len; i++)
		{
			int u = (int)(*(iter->second->A_))(i,0);
			int v = (int)(*(iter->second->A_))(i,1);
			int idx = u + (v * pCloud->width);
			inliers->indices.push_back(idx);
		}
		ROS_DEBUG("Got Indices.");

		// Extract the inliers
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		pcl::PointCloud<pcl::PointXYZ>::Ptr spCloud(new pcl::PointCloud<pcl::PointXYZ>);
		extract.setInputCloud (pCloud);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*spCloud);
		if (spCloud->size() < 1) {continue;}
		ROS_DEBUG("Got Inliers.");

		Eigen::VectorXf fitPlane;
		pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
		    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (spCloud));

		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
		ransac.setDistanceThreshold (.01);
		ransac.computeModel();
		ransac.getModelCoefficients(fitPlane);
		if (fitPlane[0] != fitPlane[0]) {continue;}
		ROS_DEBUG_STREAM("Got parameters: " << fitPlane.matrix().transpose());

		/*
		int u = (int)(*(iter->second->A_))((int)(len/2),0);
		int v = (int)(*(iter->second->A_))((int)(len/2),1);
		// Get coordinates in point cloud
		pcl::PointXYZ target = pCloud.points[u + (v * pCloud.width)];
		*/

		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*spCloud, centroid);
		if (centroid.x() != centroid.x()) {continue;}
		else
		{
			// Write id @ pixel location
			//cv::putText(overSegmented, idName, cv::Point(u,v),
			//		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,255,255), 1, CV_AA);
			//Eigen::Quaterniond q = getRotation(Eigen::Vector3d::UnitZ());
			Eigen::Quaterniond q = getRotation(Eigen::Vector3d(fitPlane[0],fitPlane[1],fitPlane[2]));


			// Generate Marker
			visualization_msgs::Marker marker;
			marker.header.frame_id = "openni_depth_optical_frame";
			//marker.header.frame_id = "world";
			marker.header.stamp = ros::Time();
			marker.ns = "MCMC";
			marker.id = count;
			marker.type = visualization_msgs::Marker::ARROW;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = centroid.x();
			marker.pose.position.y = centroid.y();
			marker.pose.position.z = centroid.z();
			marker.pose.orientation.x = q.x();//plane[0];
			marker.pose.orientation.y = q.y();//plane[1];
			marker.pose.orientation.z = q.z();//plane[2];
			marker.pose.orientation.w = q.w();
			marker.scale.x = 0.2;
			marker.scale.y = 0.2;
			marker.scale.z = 0.2;
			marker.color.a = 1.0;
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			mArray.markers.push_back(marker);
		}


	}
	superPixelPub.publish(mArray);
	outfile.close();
	//std::cout << "SuperPixels: " << count << std::endl;


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
	std::ofstream outfile;
	outfile.open("./temp/superPixels/superVectors.txt");

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
			//std::cout << j << " Plane Parameters: " << local.matrix().transpose() << std::endl;

			tempA.resize(A.rows()+superPixelIt->second->A_->rows(),superPixelIt->second->A_->cols());
			tempA << A,*(superPixelIt->second->A_);
			A = tempA;

			tempb.resize(b.rows()+superPixelIt->second->b_->rows(),superPixelIt->second->b_->cols());
			tempb << b,*(superPixelIt->second->b_);
			b = tempb;
		}

		if (A.rows() > 0)
		{
			Plane p(A,b);
			gtsam::Vector local = disparityToExplicit(p.density_.mean());
			std::cout << "Plane Parameters: " << local.matrix().transpose() << std::endl;
			std::cout << "Error: " << p.error_ << "\tNormalized: " << p.error_/A.rows() << std::endl;
			outfile << "0 0 0 " << local[0] << " " << local[1] << " " << local[2] << " " << "\n";

		}

	}
	outfile << "0 0 0 0 0 1 " << std::endl;
}
