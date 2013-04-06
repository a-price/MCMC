/**
 * \file TestGraphVisualization.cpp
 * \brief 
 *
 *  \date Apr 5, 2013
 *  \author arprice
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "GraphVisualization.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_seg");
	ros::NodeHandle nh;
	ros::Publisher nodePub;
	ros::Publisher edgePub;

	SPGraph graph;

	for(int i = 0; i < 8; i++)
	{
		SPGraph::vertex_descriptor v = boost::add_vertex(graph);

		// Creates a cube of side length size
		float size = 0.5;
		Eigen::Vector3f pos;
		pos << (size * ((i & 0x4) ? -1 : 1)),
			(size * ((i & 0x2) ? -1 : 1)),
			(size * ((i & 0x1) ? -1 : 1));

		Eigen::Vector4f theta;
		theta << pos[0],-pos[1],pos[2],1;

		std::cerr << pos.transpose() << std::endl;
		graph[v].position = pos;
		//std::cerr << graph[v].position.transpose() << std::endl;
		graph[v].modelParams = theta;

	}


	SPEdge edge;
	// Draw strong lines between top nodes
	for (int i = 0; i < 4; i++)
	{
		std::pair<SPGraph::edge_descriptor, bool> e = boost::add_edge(i, (i+1)%4, graph);
		SPGraph::edge_descriptor eID = e.first;
		graph[eID].BernoulliProbability = 0.9;
	}

	// Draw medium lines between levels
	for (int i = 0; i < 4; i++)
	{
		std::pair<SPGraph::edge_descriptor, bool> e = boost::add_edge(i, i+4, graph);
		SPGraph::edge_descriptor eID = e.first;
		graph[eID].BernoulliProbability = 0.5;
	}

	// Draw weak lines around bottom nodes
	for (int i = 0; i < 4; i++)
	{
		std::pair<SPGraph::edge_descriptor, bool> e = boost::add_edge(i+4, ((i+1)%4)+4, graph);
		SPGraph::edge_descriptor eID = e.first;
		graph[eID].BernoulliProbability = 0.2;
	}

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




