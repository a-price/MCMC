/*
 * SimpleMatcher.cpp
 *
 *  Created on: Apr 3, 2013
 *      Author: arprice
 */

/*
#include <ros/ros.h>

#include "SPGraph.h"
#include "GraphUtils.h"
#include "MatUtils.h"

//void publish3dGraph(Eigen::MatrixXf& adjacency, std::vector<Eigen::MatrixXf*> thetas, std::vector<Eigen::MatrixXf*> properties)
/*void testGraph()
{
	SPGraph graph;
	//boost::graph_traits<myGraph> viter =  boost::vertices(graph);

	SPNode node, node2;

	boost::add_vertex(node, graph);
	boost::add_vertex(node2, graph);
	//boost::add_vertex(1,graph);
	graph[0].HelloWorld = 5;
	//boost::add_vertex(1,graph);
	graph[1].HelloWorld = 10;
	graph.m_vertices.size();

	boost::add_edge(1,2,graph);
	SPGraph::vertex_descriptor vID = boost::add_vertex(node2, graph);

	graph[vID].HelloWorld = 15;

	std::cout << vID << "\t" << graph[2].HelloWorld << "\t" << graph[vID].HelloWorld << std::endl;
}*/


int main(int argc, char** argv)
{
	/*
	Eigen::MatrixXf thetaA, thetaB, adjacency;
	loadMatrix("./temp/theta1.bmat", thetaA);
	loadMatrix("./temp/theta2.bmat", thetaB);

	getPairwiseAdjacencyGraph(thetaA, thetaB, adjacency);
	//getSelfAdjacencyGraph();
	//getSelfAdjacencyGraph();

	Eigen::MatrixXf posA, posB;
	loadMatrix("pos1.bmat", posA);
	loadMatrix("pos2.bmat", posB);

	Eigen::MatrixXf propA, propB;
	loadMatrix("prop1.bmat", propA);
	loadMatrix("prop2.bmat", propB);

	visualization_msgs::MarkerArray mArray;

	int numNodes = thetaA.cols() + thetaB.cols();
	int count = 0;
	ros::Time headerTime = ros::Time::now();
	for (int i = 0; i < numNodes; i++)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "world";
		marker.header.stamp = headerTime;
		marker.ns = "MCMC";
		marker.id = count;
		marker.type = visualization_msgs::Marker::SPHERE;
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
	}*/
}

