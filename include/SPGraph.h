/**
 * \file SPGraph.h
 * \brief 
 *
 *  \date Apr 4, 2013
 *  \author Andrew Price
 */

#ifndef SPGRAPH_H_
#define SPGRAPH_H_

#include <string>
#include <Eigen/Core>
#include <boost/graph/adjacency_list.hpp>
#include <boost/serialization/string.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/connected_components.hpp>

#include "Graph.h"

namespace boost
{
template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline void serialize(
	Archive & ar,
	Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & t,
	const unsigned int file_version)
{
    size_t rows = t.rows(), cols = t.cols();
    ar & rows;
    ar & cols;
    if( rows * cols != t.size() )
    t.resize( rows, cols );

    for(size_t i=0; i<t.size(); i++)
    	ar & t.data()[i];
}
}

class SPNode
{
public:
	size_t spid;
	size_t currentSegmentID;
	int numPixels;
	std::string parentFrame;
	Eigen::Vector4f modelParams;
	Eigen::Vector3f position;
	Eigen::Vector2i imagePosition;

	Eigen::VectorXf getFullModel()
	{
		Eigen::VectorXf retVal(modelParams.rows()+position.rows());
		retVal << modelParams, position;
		return retVal;
	}

	Eigen::VectorXf getDefaultWeights()
	{
		Eigen::VectorXf retVal(modelParams.rows()+position.rows());
		retVal << 5.0, 5.0 ,5.0, 10.0,
				0.5, 0.5, 0.5;
		return retVal;
	}

	template<class Archive>
	void serialize(Archive &ar, const unsigned int version)
	{
		ar & spid;
		ar & numPixels;
		ar & parentFrame;
		boost::serialize(ar, modelParams, version);
		boost::serialize(ar, position, version);
		boost::serialize(ar, imagePosition, version);
	}
};

class SPEdge
{
public:
	double BernoulliProbability;
	bool partitionOn;

	template<class Archive>
	void serialize(Archive &ar, const unsigned int version)
	{
		ar & BernoulliProbability;
		ar & partitionOn;
	}
};

typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, SPNode, SPEdge> SPGraph;

class SPEdgePredicate
{
public:
	SPEdgePredicate() : mGraph(0) {}
	SPEdgePredicate(SPGraph& graph) : mGraph(&graph) {}

	bool operator() (const SPGraph::edge_descriptor edgeID) const
	{
		return (*mGraph)[edgeID].partitionOn;
	}
private:
	SPGraph* mGraph;
};

typedef boost::filtered_graph<SPGraph, SPEdgePredicate> SPFilteredGraph;

#endif /* SPGRAPH_H_ */
