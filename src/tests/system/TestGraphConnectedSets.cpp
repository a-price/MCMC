/**
 * \file TestGraphConnectedSets.cpp
 * \brief 
 *
 *  \date Apr 10, 2013
 *  \author arprice
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

	std::cerr << "started.\n";
	SPGraph graph;// = generateSampleGraph();
	SPGraph graph1 = generateSampleGraph();
	SPGraph graph2 = generateSampleGraph();

	std::cerr << "got sample graphs.\n";
	SPGraph::vertex_iterator vertexItA, vertexEndA;
	boost::tie(vertexItA, vertexEndA) = boost::vertices(graph2);
	for (; vertexItA != vertexEndA; ++vertexItA)
	{
		graph2[*vertexItA].position[0]++;
		graph2[*vertexItA].position[1]++;
	}


	std::cerr << "about to merge.\n";
	mergeNewScanGraph(graph, graph1);
	mergeNewScanGraph(graph, graph2);

	std::cerr << "merged.\n";
	nodePub = nh.advertise<visualization_msgs::MarkerArray>( "graph_nodes", 0 );
	edgePub = nh.advertise<visualization_msgs::MarkerArray>( "graph_edges", 0 );

	std::vector<visualization_msgs::MarkerArray> mArrays;


	ros::Rate rate(.25);
	while(ros::ok())
	{
		getNewConnectedSets(graph);
		mArrays = GraphVisualization::VisualizeGraph(graph);

		nodePub.publish(mArrays[0]);
		edgePub.publish(mArrays[1]);

		ros::spinOnce();
		rate.sleep();
	}
}




