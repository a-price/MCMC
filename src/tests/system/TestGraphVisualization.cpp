/**
 * \file TestGraphVisualization.cpp
 * \brief 
 *
 * \date Apr 5, 2013
 * \author arprice
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "GraphVisualization.h"
#include "GraphUtils.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_seg");
	ros::NodeHandle nh;
	ros::Publisher nodePub;
	ros::Publisher edgePub;

	SPGraph graph = generateSampleGraph();

	nodePub = nh.advertise<visualization_msgs::MarkerArray>( "graph_nodes", 0 );
	edgePub = nh.advertise<visualization_msgs::MarkerArray>( "graph_edges", 0 );

	std::vector<visualization_msgs::MarkerArray> mArrays;
	mArrays = GraphVisualization::VisualizeGraph(graph);
	ROS_ASSERT(mArrays[1].markers.size() == 4);
	ROS_ASSERT(mArrays[1].markers[0].points.size() == 4);

	ros::Rate rate(.25);
	while(ros::ok())
	{
		nodePub.publish(mArrays[0]);
		edgePub.publish(mArrays[1]);
		ros::spinOnce();
		rate.sleep();
	}
}




