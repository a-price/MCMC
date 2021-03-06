/**
 * \file GraphVisualization.cpp
 * \brief 
 *
 * \date Apr 5, 2013
 * \author arprice
 */

#include "GraphVisualization.h"

GraphVisualization::GraphVisualization()
{
	nodePub = nh.advertise<visualization_msgs::MarkerArray>( "graph_nodes_iter", 0 );
	edgePub = nh.advertise<visualization_msgs::MarkerArray>( "graph_edges_iter", 0 );

	cloudPub = nh.advertise<sensor_msgs::PointCloud2>( "cloud_seg_iter", 0 );
}

GraphVisualization::~GraphVisualization()
{
	// TODO Auto-generated destructor stub
}

void GraphVisualization::VisualizeGraphStep(SPGraph& graph, std::set<SPGraph::vertex_descriptor>& proposedComponent, MultiviewSegmentation& segmentation)
{
	/********** Generate Node Markers **********/
	visualization_msgs::MarkerArray thetas;
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	sensor_msgs::PointCloud2 pcMsg;
	SPGraph::vertex_iterator vertexIt, vertexEnd;
	boost::tie(vertexIt, vertexEnd) = boost::vertices(graph);
	for (; vertexIt != vertexEnd; ++vertexIt)
	{
		SPGraph::vertex_descriptor vertexID = *vertexIt; // dereference vertexIt, get the ID
		SPNode & vertex = graph[vertexID];
		MultiviewSegment* segment = segmentation.segmentsMap.find( vertex.currentState->currentSegmentID)->second;

		Eigen::Quaternionf q;
		q.setFromTwoVectors(Eigen::Vector3f(
				vertex.modelParams[0],vertex.modelParams[1],
				vertex.modelParams[2]), Eigen::Vector3f::UnitX());

		// Create a marker to show parameters
		visualization_msgs::Marker marker;
		marker.header.frame_id = "world";
		//marker.header.stamp = headerTime;
		marker.ns = "MCMC";
		marker.id = (int)vertexID;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = vertex.position[0];
		marker.pose.position.y = vertex.position[1];
		marker.pose.position.z = vertex.position[2];
		marker.pose.orientation.x = q.x();
		marker.pose.orientation.y = q.y();
		marker.pose.orientation.z = q.z();
		marker.pose.orientation.w = q.w();
		marker.scale.x = 0.02;
		marker.scale.y = 0.02;
		marker.scale.z = 0.02;
		marker.color.a = 1.0; // based on # & weight of connections

		if (proposedComponent.find(vertexID) != proposedComponent.end())
		{
			marker.color.r = 1.0; //
			marker.color.g = 1.0; //
			marker.color.b = 0.0;
		}
		else
		{
			marker.color.r = segment->r; //
			marker.color.g = segment->g; //
			marker.color.b = segment->b;
		}
		thetas.markers.push_back(marker);

		// Add points from segment to point cloud
		for(int p = 0; p < vertex.subCloud->points.size(); p++)
		{
			pcl::PointXYZRGB point;
			point.x = vertex.subCloud->points[p].x;
			point.y = vertex.subCloud->points[p].y;
			point.z = vertex.subCloud->points[p].z;
			point.r = segment->r;
			point.g = segment->g;
			point.b = segment->b;
			cloud.points.push_back(point);
		}
	}

	/********** Generate Edge Markers **********/
	visualization_msgs::MarkerArray mus;
	int idIdx = 0;
	SPGraph::edge_iterator edgeIt, edgeEnd;
	boost::tie(edgeIt, edgeEnd) = boost::edges(graph);
	for (; edgeIt != edgeEnd; ++edgeIt)
	{
		SPGraph::edge_descriptor edgeID = *edgeIt; // dereference edgeIt, get the ID
		SPEdge & edge = graph[edgeID];

		SPGraph::vertex_descriptor aID, bID;
		aID = boost::source(edgeID, graph);
		bID = boost::target(edgeID, graph);
		SPNode & a = graph[aID];
		SPNode & b = graph[bID];

		geometry_msgs::Point pa, pb;
		pa.x=a.position[0]; pa.y=a.position[1]; pa.z=a.position[2];
		pb.x=b.position[0]; pb.y=b.position[1]; pb.z=b.position[2];

		visualization_msgs::Marker eMarker;
		eMarker.header.frame_id = "/world";
		eMarker.ns = "MCMC";
		eMarker.id = idIdx++;
		eMarker.type = visualization_msgs::Marker::ARROW;

		if (edge.currentState->partitionOn)
		{
			eMarker.action = visualization_msgs::Marker::ADD;
		}
		else
		{
			eMarker.action = visualization_msgs::Marker::DELETE;
		}

		if (proposedComponent.find(aID) != proposedComponent.end() &&
			proposedComponent.find(bID) != proposedComponent.end())
		{
			eMarker.color.r = 0.75; //
			eMarker.color.g = 0.75; //
			eMarker.color.b = 0.0;
			eMarker.color.a = 1.0;
		}
		else
		{
			eMarker.color.r = 0.05; //
			eMarker.color.g = 0.05; //
			eMarker.color.b = 0.05;
			eMarker.color.a = 0.75;
		}


		eMarker.points.push_back(pa);
		eMarker.points.push_back(pb);

		eMarker.scale.z = 0; // remove head?

		float lineScale = 0.025;

		eMarker.scale.x = lineScale * edge.BernoulliProbability;
		eMarker.scale.y = eMarker.scale.x;
		mus.markers.push_back(eMarker);
	}

	pcl::toROSMsg(cloud, pcMsg);
	pcMsg.header.frame_id="/world";

	nodePub.publish(thetas);
	edgePub.publish(mus);
	cloudPub.publish(pcMsg);
}


// TODO: Add filters for small superpixels and weak edges
std::vector<visualization_msgs::MarkerArray> GraphVisualization::VisualizeGraph(SPGraph& graph)
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
		pa.x=a.position[0]; pa.y=a.position[1]; pa.z=a.position[2];
		pb.x=b.position[0]; pb.y=b.position[1]; pb.z=b.position[2];

		visualization_msgs::Marker eMarker;
		eMarker.header.frame_id = "/world";
		eMarker.ns = "MCMC";
		eMarker.id = idIdx++;
		eMarker.type = visualization_msgs::Marker::ARROW;

		if (edge.currentState->partitionOn)
		{
			eMarker.action = visualization_msgs::Marker::ADD;
		}
		else
		{
			eMarker.action = visualization_msgs::Marker::DELETE;
		}

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
		}
		else if (edge.BernoulliProbability <= 0.75 && edge.BernoulliProbability > 0.25)
		{
			eMarker.scale.x = lineScale * 0.5;
		}
		else if (edge.BernoulliProbability <= 0.25 && edge.BernoulliProbability > 0.05)
		{
			eMarker.scale.x = lineScale * 0.25;
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

