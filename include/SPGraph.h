/**
 * \file SPGraph.h
 * \brief 
 *
 *  \date Apr 4, 2013
 *  \author arprice
 */

#ifndef SPGRAPH_H_
#define SPGRAPH_H_

#include <string>
#include <Eigen/Core>
#include <boost/graph/adjacency_list.hpp>

#include "Graph.h"


class SPNode
{
public:
	std::string parentFrame;
	Eigen::Vector4f modelParams;
	Eigen::Vector3f position;
	Eigen::Vector2i imagePosition;
	int numPixels;
	size_t spid;

	Eigen::VectorXf getFullModel()
	{
		Eigen::VectorXf retVal;
		retVal << modelParams, position;
		return retVal;
	}

	static Eigen::VectorXf getDefaultWeights()
	{
		Eigen::VectorXf retVal;
		retVal << 1.0,1.0,1.0,4.0,
				0.25,0.25,0.25;
		return retVal;
	}
};

class SPEdge
{
public:
	double BernoulliProbability;
};

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, SPNode, SPEdge> SPGraph;


class SPGraphManager
{
public:
	SPGraph spGraph;
	std::map<SuperPixelID, SPGraph::vertex_descriptor> idLookup;

};


#endif /* SPGRAPH_H_ */
