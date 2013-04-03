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

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

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

// Segmentation includes...
//#include "InteractiveSegmenter.h"
#include "SegmentationContext.h"
/*
#include "Common.h"
#include "Graph.h"
#include "IO.h"
#include "OverSegmentation.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>

#include <map>
*/
#include <math.h>
#include <exception>
#include "tf_eigen.h"
#include "GraphUtils.h"
#include "MatUtils.h"
// Visualization stuff

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image, sensor_msgs::PointCloud2> KinectSyncPolicy;

const std::string bagFilename = "/media/Data/RGBD-Datasets/Freiburg/rgbd_dataset_freiburg3_cabinet-2hz-with-pointclouds.bag";
const std::string paramFilename = "/home/arprice/fuerte_workspace/sandbox/MCMC/bin/overSegmentationParams.txt";
const std::string colorTopic = "/camera/rgb/image_color";
const std::string depthTopic = "/camera/depth/image";
const std::string pointTopic = "/camera/rgb/points";

ros::Publisher superPixelPub;
ros::Publisher hyperPixelPub;
ros::Publisher segCloudPub;
tf::TransformListener* listener;
tf::TransformBroadcaster* broadcaster;
std::vector<SegmentationContext> contexts;
volatile int contextNum = -1;

OverSegmentationParameters params;

void kinectCallback(const sensor_msgs::ImageConstPtr color, const sensor_msgs::ImageConstPtr depth, const sensor_msgs::PointCloud2ConstPtr points);
void segment(cv::Mat disparities, cv::Mat colors, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, Eigen::Isometry3d cameraPose);
void createLookup(const Graph& graph, Eigen::MatrixXf& lookup);

Eigen::Quaterniond getRotation(Eigen::Vector3d axis, Eigen::Vector3d initial)
{
	// Assume @ (1,0,0) originally
	if (axis.norm() - 1.0 > 0.001) {axis.normalize();}
	Eigen::Vector3d q = initial.cross(axis);
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
	ros::Time::init();

	IO::readSegmentationParams(paramFilename, params);
	params.print();

#ifndef USE_BAGFILE
	cv::Mat disparities, colors, depth3;

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

	listener = (new tf::TransformListener());
	broadcaster = (new tf::TransformBroadcaster());

	ros::spinOnce();

	superPixelPub = nh_.advertise<visualization_msgs::MarkerArray>( "superpixel_vectors", 0 );
	hyperPixelPub = nh_.advertise<visualization_msgs::MarkerArray>( "hyperpixel_vectors", 0 );
	segCloudPub = nh_.advertise<sensor_msgs::PointCloud2>("color_cloud", 0);

	message_filters::Subscriber<sensor_msgs::Image> color_sub_ (nh_, colorTopic, 8);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub_ (nh_, depthTopic, 8);
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_ (nh_, pointTopic, 8);
	message_filters::Synchronizer<KinectSyncPolicy> sync_(KinectSyncPolicy(8), color_sub_, depth_sub_, cloud_sub_);

	sync_.registerCallback(boost::bind(&kinectCallback, _1, _2, _3));

	//ros::CallbackQueue my_queue;
	//nh_.setCallbackQueue(&my_queue);
	//my_queue.callOne(ros::WallDuration());
	//ROS_INFO("Finished");
	//return 0;
	ros::spin();
#endif

}



const int maxCallbacks = 5;
int numCallbacks = 0;
void kinectCallback(const sensor_msgs::ImageConstPtr color, const sensor_msgs::ImageConstPtr depth, const sensor_msgs::PointCloud2ConstPtr points)
{
	ROS_INFO("Got Callback. Sensor Frame: %s", points->header.frame_id.c_str());
	if (numCallbacks < maxCallbacks && ros::ok())
	{
		numCallbacks++;
	}
	else
	{
		//return;
	}
	cv::Mat disparities, colors, depth3;

	cv_bridge::CvImagePtr cPtr = cv_bridge::toCvCopy(color, "bgr8");
	cv_bridge::CvImagePtr dPtr = cv_bridge::toCvCopy(depth, "mono8");
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud (new pcl::PointCloud<pcl::PointXYZ>); // no color in this topic?
	pcl::fromROSMsg(*points, *pCloud);

	tf::StampedTransform transform;
	Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
	try
	{
		listener->waitForTransform("/world", ros::Time(0),
				points->header.frame_id, points->header.stamp,
				"/world", ros::Duration(2.0));
		listener->lookupTransform("/world", ros::Time(0),
				points->header.frame_id, points->header.stamp,
				"/world", transform);
		tf::TransformTFToEigen(transform, pose);
	}
	catch(std::exception& ex)
	{
		ROS_ERROR("TF Lookup Failed: %s", ex.what());
	}

	ROS_INFO("Got Cloud.");

	// TODO:convert to disparity?
	dPtr->image.copyTo(disparities);
	cPtr->image.copyTo(colors);
	segment(disparities, colors, pCloud, pose);
	//segment(disparities, colors, pcl::PointCloud<pcl::PointXYZ>::Ptr(&pCloud), pose);
	//segment(*disparityImage(dPtr->image), cPtr->image, pcl::PointCloud<pcl::PointXYZ>::Ptr(&pCloud));

}

void segment(cv::Mat d, cv::Mat c,
		     pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
		     Eigen::Isometry3d cameraPose) //, cv::Mat cd)
{
	ROS_INFO("Called Segment.");
	ros::Time headerTime = ros::Time::now();
	tf::StampedTransform transform;
	tf::TransformEigenToTF(cameraPose, transform);
	Eigen::Isometry3f cameraFrameTF = cameraPose.cast<float>();//cameraPose.inverse(Eigen::TransformTraits::Isometry).cast<float>();
	broadcaster->sendTransform(tf::StampedTransform(transform, headerTime, "/world", "segment_result"));

	srand(time(NULL));
	cv::Mat disparities, colors, filtd, filtc, filtcd;
	c.copyTo(colors);
	d.copyTo(disparities);

	size_t window = params.windowSize_;

	for(int i = 0; i < 2; i++)
	{
		cv::bilateralFilter(colors, filtc, window, window*2, window/2);
		filtc.copyTo(colors);

		cv::bilateralFilter(disparities, filtd, window, window*2, window);
		filtd.copyTo(disparities);
	}

	int res = system("mkdir -p ./temp/superPixels/");
	cv::imwrite("./temp/superPixels/colorf.png", colors);

	// Segmentation setup
	//SegmentationContext sc;
	Graph graph;
	cv::Mat overSegmented;
	Eigen::MatrixXf spLookup;
	std::map<SuperPixelID, int> SPtoS;
	std::vector<SuperPixelID> StoSP[4];

	// Segmentation call

	gttic_(OverSegmentation);
	ROS_INFO("Segmenting.");
	OverSegmentation::overSegment(disparities, colors, params, graph);
	gttoc_(OverSegmentation);
	tictoc_print_();
	ROS_INFO("Got Segmentation.");

	// Get the color image
	OverSegmentation::visualize(graph, overSegmented);
	createLookup(graph, spLookup);
	cv::imwrite("./temp/superPixels/overSegmented.png", overSegmented);
	ROS_INFO("Generated Lookup.");

	pcl::PointCloud<pcl::PointXYZRGB> outCloud;
	std::map<SuperPixelID, SuperPixel*>::iterator iter;
	std::string idName;
	int count = 0, validCount = 0;

	visualization_msgs::MarkerArray mArray;
	std::map<long unsigned int, int> spModelLookup;
	std::vector<Eigen::Vector2i> spCenters;
	Eigen::MatrixXf Theta = Eigen::MatrixXf::Zero(4, graph.superPixels_.size());

	ROS_INFO("Got SPs.");
	for (iter = graph.superPixels_.begin(); iter != graph.superPixels_.end(); ++iter)
	{
		count++;
		//Eigen::Vector4d plane = disparityToExplicit(iter->second->plane_->density_.mean());
		//plane.normalize();

		// Get ID
		idName = std::to_string((long unsigned int)(iter->first));

		ROS_DEBUG("Getting Indices.");
		// Get central pixel (middle?)
		int len = iter->second->A_->rows();
		if (len < 1) {continue;}
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
		for (int i = 0; i < len; i++)
		{
			int u = (int)(*(iter->second->A_))(i,0);
			int v = (int)(*(iter->second->A_))(i,1);

			int idx = u + (v * pCloud->width);
			inliers->indices.push_back(idx);


			//cv::Vec3f pixel = colors.at<cv::Vec3f>(v,u);
			cv::Vec3b pixel = overSegmented.at<cv::Vec3b>(v,u);
			pcl::PointXYZ point = pCloud->points[idx];
			pcl::PointXYZRGB sPoint;
			sPoint.x = point.x; sPoint.y = point.y; sPoint.z = point.z;
			sPoint.b = pixel[0]; sPoint.g = pixel[1]; sPoint.r = pixel[2];
			outCloud.points.push_back(sPoint);
		}
		ROS_DEBUG("Got Indices.");

		/********** Get Subset **********/
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		pcl::PointCloud<pcl::PointXYZ>::Ptr spCloud(new pcl::PointCloud<pcl::PointXYZ>);
		extract.setInputCloud (pCloud);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*spCloud);
		ROS_DEBUG("Got Inliers.");

		/********** Generate Plane Params **********/
		Eigen::VectorXf fitPlane;
		pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
		    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (spCloud));

		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
		ransac.setDistanceThreshold (.01);
		ransac.computeModel();
		ransac.getModelCoefficients(fitPlane);
		//ransac.getInliers()

		if (fitPlane[0] != fitPlane[0]) {continue;}
		if (fabs(fitPlane[0]) < 0.001) {continue;}
		if (fitPlane[3] < -0.5) {fitPlane = -fitPlane;}

		// Convert to Globals
		Eigen::VectorXf theta(4);
		theta.topRows(3) = cameraFrameTF.rotation() * fitPlane.topRows(3);
		theta(3) = fitPlane(3);

		Theta.col(validCount) = theta;//fitPlane;
		spModelLookup.insert(std::pair<long unsigned int, int>(iter->first, validCount));
		validCount++;
		//ROS_DEBUG_STREAM("Got parameters: " << fitPlane.matrix().transpose());


		int u = (int)(*(iter->second->A_))((int)(len/2),0);
		int v = (int)(*(iter->second->A_))((int)(len/2),1);
		spCenters.push_back(Eigen::Vector2i(u,v));
		// Get coordinates in point cloud
		//pcl::PointXYZ target = pCloud.points[u + (v * pCloud.width)];

		/********** Generate Centroid **********/
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*spCloud, centroid);
		if (centroid.x() != centroid.x()) {continue;}

		Eigen::Vector4f gCentroid;
		if (centroid(3) == 0) {centroid(3) = 1;}
		gCentroid = cameraFrameTF * centroid;

		//ROS_INFO_STREAM("centroid:\n" << centroid.matrix().transpose()
		//		<< "gCentroid:\n" << gCentroid.matrix().transpose());

		// Write id @ pixel location
		//cv::putText(overSegmented, idName, cv::Point(u,v),
		//		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,255,255), 1, CV_AA);
		//Eigen::Quaterniond q = getRotation(Eigen::Vector3d::UnitZ());
		//Eigen::Quaterniond q = getRotation(Eigen::Vector3d(fitPlane[0],fitPlane[1],fitPlane[2]), Eigen::Vector3d::UnitX());
		Eigen::Quaterniond q = getRotation(Eigen::Vector3d(theta[0],theta[1],theta[2]), Eigen::Vector3d::UnitX());


		/********** Generate Marker **********/
		visualization_msgs::Marker marker;
		marker.header.frame_id = "world";
		marker.header.stamp = headerTime;
		marker.ns = "MCMC";
		marker.id = count;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = gCentroid.x();
		marker.pose.position.y = gCentroid.y();
		marker.pose.position.z = gCentroid.z();
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

	// Trim Theta
	Theta.conservativeResize(4, validCount);
	saveMatrix("theta2.bmat", Theta);
	//ROS_INFO_STREAM("Theta: \n" << Theta.matrix());

	// Save distances
	Eigen::MatrixXf distances = Theta.row(3);
	ROS_INFO_STREAM(cameraFrameTF.matrix());

	Eigen::VectorXf weights(4);
	weights << 1,1,1,4;
	Eigen::MatrixXf g = getPlanarAdjacencyGraph(graph, Theta, weights, spModelLookup);//getSelfAdjacencyGraph(Theta, 0.5);
	//ROS_INFO_STREAM("Adjacency Graph: \n" << g);
	writeOrderedGraph(g, "graph.dot", spCenters);

	ROS_INFO("Ready to publish.");
	sensor_msgs::PointCloud2 msgCloud;
	pcl::toROSMsg(outCloud, msgCloud);
	//msgCloud.header.seq = 1;
	msgCloud.header.frame_id = "segment_result";
	msgCloud.header.stamp = headerTime;


	//ROS_INFO("Cloud Width: %i\tHeight: %i", msgCloud.width, msgCloud.height);
	superPixelPub.publish(mArray);
	segCloudPub.publish(msgCloud);
	ROS_INFO("Published.");

	//contexts.push_back(sc);

	//cv::namedWindow("Result");
	//cv::setMouseCallback("Result", onMouse, 0);
	//cv::imshow("Result", overSegmented);

	//cv::waitKey(0);

	//TODO: Better way for this...
	//numCallbacks--;

	ROS_INFO("Unto the Breach!.");
	return;
}

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


