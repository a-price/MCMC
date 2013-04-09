/**
 * \file GraphVisualization.cpp
 * \brief 
 *
 *  \date Apr 5, 2013
 *  \author arprice
 */

#include "GraphVisualization.h"

GraphVisualization::GraphVisualization()
{
	// TODO Initialize color Array

}

GraphVisualization::~GraphVisualization()
{
	// TODO Auto-generated destructor stub
}


// TODO: Add filters for small superpixels and weak edges
std::vector<visualization_msgs::MarkerArray> GraphVisualization::VisualizeGraph(SPGraph graph)
{
	std::vector<visualization_msgs::MarkerArray> mArrays;

	/********** Generate Node Markers **********/
	visualization_msgs::MarkerArray thetas;
	SPGraph::vertex_iterator vertexIt, vertexEnd;
	boost::tie(vertexIt, vertexEnd) = boost::vertices(graph);
	for (; vertexIt != vertexEnd; ++vertexIt)
	{
		SPGraph::vertex_descriptor vertexID = *vertexIt; // dereference vertexIt, get the ID
		SPNode & vertex = graph[vertexID];

		//std::cout << vertex.position.transpose() << std::endl;

		Eigen::Quaternionf q;
		q.setFromTwoVectors(Eigen::Vector3f(
				vertex.modelParams[0],vertex.modelParams[1],
				vertex.modelParams[2]), Eigen::Vector3f::UnitX());

		// Get color by reference frame

		visualization_msgs::Marker marker;
		marker.header.frame_id = "world";
		//marker.header.stamp = headerTime;
		marker.ns = "MCMC";
		marker.id = (int)vertexID;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = vertex.position[0];
		marker.pose.position.y = vertex.position[1];
		marker.pose.position.z = vertex.position[2];
		marker.pose.orientation.x = q.x();
		marker.pose.orientation.y = q.y();
		marker.pose.orientation.z = q.z();
		marker.pose.orientation.w = q.w();
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 0.75; // based on # & weight of connections

		marker.color.r = 0.0; // based on parent
		marker.color.g = 1.0; //
		marker.color.b = 0.0;
		thetas.markers.push_back(marker);
	}
	mArrays.push_back(thetas);



	/********** Generate Edge Markers **********/
	/*
	visualization_msgs::Marker strong;
	strong.header.frame_id = "world";
	strong.ns = "MCMC";
	strong.type = visualization_msgs::Marker::SPHERE_LIST;
	strong.action = visualization_msgs::Marker::ADD;
	//strong.pose.position.x = 1;
	//strong.pose.position.y = 0;
	//strong.pose.position.z = 0;
	strong.color.r = 0.75; // based on parent
	strong.color.g = 0.75; //
	strong.color.b = 0.0;

	visualization_msgs::Marker moderate(strong);
	visualization_msgs::Marker weak(strong);
	visualization_msgs::Marker faint(strong);

	strong.scale.x = 1;
	moderate.scale.x = .5;
	weak.scale.x = .25;
	faint.scale.x = .1;

	strong.id=50;
	moderate.id=51;
	weak.id=52;
	faint.id=53;

	strong.color.a = 0.9;
	moderate.color.a = 0.5;
	weak.color.a = 0.25;
	faint.color.a = 0.1;


	SPGraph::edge_iterator edgeIt, edgeEnd;
	boost::tie(edgeIt, edgeEnd) = boost::edges(graph);
	for (; edgeIt != edgeEnd; ++edgeIt)
	{
		SPGraph::edge_descriptor edgeID = *edgeIt; // dereference edgeIt, get the ID
		SPEdge & edge = graph[edgeID];

		SPNode & a = graph[boost::source(edgeID, graph)];
		SPNode & b = graph[boost::target(edgeID, graph)];

		geometry_msgs::Point pa, pb;
		std::cout << a.position.transpose() << "\t" << b.position.transpose() << std::endl;
		pa.x=a.position[0]; pa.x=a.position[1]; pa.x=a.position[2];
		pb.x=b.position[0]; pb.x=b.position[1]; pb.x=b.position[2];

		// We have to use a line list marker here, so all lines are part of the same marker
		if (edge.BernoulliProbability > 0.75)
		{
			strong.points.push_back(pa);
			strong.points.push_back(pb);
			std::cout << "Strong!" << std::endl;
		}
		else if (edge.BernoulliProbability <= 0.75 && edge.BernoulliProbability > 0.25)
		{
			moderate.points.push_back(pa);
			moderate.points.push_back(pb);
			std::cout << "Moderate!" << std::endl;
		}
		else if (edge.BernoulliProbability <= 0.25 && edge.BernoulliProbability > 0.05)
		{
			weak.points.push_back(pa);
			weak.points.push_back(pb);
			std::cout << "Weak!" << std::endl;
		}
		else if (edge.BernoulliProbability <= 0.05)
		{
			faint.points.push_back(pa);
			faint.points.push_back(pb);
		}
	}
	visualization_msgs::MarkerArray lines;
	lines.markers.push_back(strong);
	lines.markers.push_back(moderate);
	lines.markers.push_back(weak);
	lines.markers.push_back(faint);*/

	visualization_msgs::MarkerArray qs;
	int idIdx = 0;
	SPGraph::edge_iterator edgeIt, edgeEnd;
	boost::tie(edgeIt, edgeEnd) = boost::edges(graph);
	for (; edgeIt != edgeEnd; ++edgeIt)
	{
		SPGraph::edge_descriptor edgeID = *edgeIt; // dereference edgeIt, get the ID
		SPEdge & edge = graph[edgeID];

		SPNode & a = graph[boost::source(edgeID, graph)];
		SPNode & b = graph[boost::target(edgeID, graph)];

		geometry_msgs::Point pa, pb;
		std::cout << a.position.transpose() << "\t" << b.position.transpose() << std::endl;
		pa.x=a.position[0]; pa.y=a.position[1]; pa.z=a.position[2];
		pb.x=b.position[0]; pb.y=b.position[1]; pb.z=b.position[2];

		visualization_msgs::Marker eMarker;
		eMarker.header.frame_id = "world";
		eMarker.ns = "MCMC";
		eMarker.id = idIdx++;
		eMarker.type = visualization_msgs::Marker::ARROW;
		eMarker.action = visualization_msgs::Marker::ADD;

		eMarker.color.r = 0.75; //
		eMarker.color.g = 0.75; //
		eMarker.color.b = 0.0;
		eMarker.color.a = 0.75;
		eMarker.points.push_back(pa);
		eMarker.points.push_back(pb);

		eMarker.scale.z = 0; // remove head?

		float lineScale = 0.025;
		// We have to use a line list marker here, so all lines are part of the same marker
		if (edge.BernoulliProbability > 0.75)
		{
			eMarker.scale.x = lineScale * 1.0;
			std::cout << "Strong!" << std::endl;
		}
		else if (edge.BernoulliProbability <= 0.75 && edge.BernoulliProbability > 0.25)
		{
			eMarker.scale.x = lineScale * 0.5;
			std::cout << "Moderate!" << std::endl;
		}
		else if (edge.BernoulliProbability <= 0.25 && edge.BernoulliProbability > 0.05)
		{
			eMarker.scale.x = lineScale * 0.25;
			std::cout << "Weak!" << std::endl;
		}
		else if (edge.BernoulliProbability <= 0.05)
		{
			eMarker.scale.x = lineScale * 0.12;
		}
		eMarker.scale.y = eMarker.scale.x;
		qs.markers.push_back(eMarker);
	}


	mArrays.push_back(qs);

	return mArrays;
}

