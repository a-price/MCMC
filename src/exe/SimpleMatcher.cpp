/*
 * SimpleMatcher.cpp
 *
 *  Created on: Apr 3, 2013
 *      Author: arprice
 */

#include <iostream>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/archive/text_iarchive.hpp>

#include "SegmentationContext.h"

#include "SPGraph.h"
#include "GraphUtils.h"
#include "MatUtils.h"
#include "GraphVisualization.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_seg");
	ros::NodeHandle nh;

	ros::Publisher nodePub;
	ros::Publisher edgePub;

	nodePub = nh.advertise<visualization_msgs::MarkerArray>( "graph_nodes_iter", 0 );
	edgePub = nh.advertise<visualization_msgs::MarkerArray>( "graph_edges_iter", 0 );

	SPGraph spGraph;

	std::ifstream ifs("test.big");
	{
		boost::archive::text_iarchive ia(ifs);
		// write class instance to archive
		ia >> spGraph;
		// archive and stream closed when destructors are called
	}

	ros::Rate rate(0.5);
	while(ros::ok())
	{
		std::cout << "Beginning SW.\n";
		getNewConnectedSets(spGraph);

		std::vector<visualization_msgs::MarkerArray> mArrays;
		mArrays = GraphVisualization::VisualizeGraph(spGraph);

		nodePub.publish(mArrays[0]);
		edgePub.publish(mArrays[1]);

		std::cout << "New Partition.\n";
		ros::spinOnce();
		std::cerr << "Going to sleep for a bit...";
		rate.sleep();
		std::cerr << "done.\n";
	}
}

