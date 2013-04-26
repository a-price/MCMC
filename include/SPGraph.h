/**
 * \file SPGraph.h
 * \brief 
 *
 *  \date Apr 4, 2013
 *  \author Andrew Price
 */

#ifndef SPGRAPH_H_
#define SPGRAPH_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/common/centroid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "Common.h"
#include <string>
#include <Eigen/Core>
#include <boost/graph/adjacency_list.hpp>
#include <boost/serialization/string.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/graph/filtered_graph.hpp>

#include "MathUtils.h"

class SPNodeState
{
public:
	size_t currentSegmentID;
	size_t currentModel;
	Eigen::Vector4f currentModelParams;
};

class SPEdgeState
{
public:
	bool partitionOn;
};

class SPNode
{
public:
	size_t spid;
	std::string parentFrame;
	Eigen::Vector4f modelParams;
	Eigen::Vector3f position;
	Eigen::Vector2i imagePosition;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr subCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr subCloudCartesian;

	boost::shared_ptr<SPNodeState> currentState;
	boost::shared_ptr<SPNodeState> proposedState;

	SPNode();

	void acceptProposedState();

	void setSubCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud);

	void computeFullModel();
	Eigen::VectorXf getFullModel();
	Eigen::VectorXf getDefaultWeights();

	double getErrorForProposedModel();
	double getErrorForPlaneModel(Eigen::Vector4f& planeCoefficients);
};

class SPEdge
{
public:
	double BernoulliProbability;

	boost::shared_ptr<SPEdgeState> currentState;
	boost::shared_ptr<SPEdgeState> proposedState;

	SPEdge();
//	~SPEdge();

	void acceptProposedState();
};

typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, SPNode, SPEdge> SPGraph;

class SPEdgePredicate
{
public:
	SPEdgePredicate() : mGraph(0) {}
	SPEdgePredicate(SPGraph& graph) : mGraph(&graph) {}

	bool operator() (const SPGraph::edge_descriptor edgeID) const;

private:
	SPGraph* mGraph;
};

typedef boost::filtered_graph<SPGraph, SPEdgePredicate> SPFilteredGraph;

#endif /* SPGRAPH_H_ */
