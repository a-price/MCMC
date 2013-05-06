/**
 * \file MultiviewSegmentation.h
 * \brief 
 *
 *  \date Apr 17, 2013
 *  \author Andrew Price
 */

#ifndef MULTIVIEWSEGMENTATION_H_
#define MULTIVIEWSEGMENTATION_H_

#include <iostream>
#include <boost/graph/connected_components.hpp>
#include "MultiviewSegment.h"

class SegmentGraphNode
{
public:
	int r,g,b;
	long segmentID;
};

typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, SegmentGraphNode, boost::no_property> SegmentGraph;

class MultiviewSegmentation
{
public:

	enum STATE_TYPE
	{
		ACCEPTED,
		PROPOSED
	};

	SPGraph& mGraph;
	//const SPFilteredGraph& mFGraph;

	std::vector<MultiviewSegment> segments;
	std::map<long, MultiviewSegment*> segmentsMap;

	double probability;

	STATE_TYPE isProposal;

	MultiviewSegmentation (SPGraph& graph);

	MultiviewSegment* getParentSegment(SPGraph::vertex_descriptor target);

	MultiviewSegment* addNewSegment(std::set<SPGraph::vertex_descriptor> elements);

	std::set<MultiviewSegment*> getNeighborSegments(std::set<SPGraph::vertex_descriptor> elements);

	SegmentGraph getSegmentGraph();

	void colorSegmentGraph(SegmentGraph& sGraph);

	void moveSuperpixels(std::set<SPGraph::vertex_descriptor> elements, MultiviewSegment& SA1, MultiviewSegment& SA2, MultiviewSegment& SB1, MultiviewSegment& SB2);

	void getNewConnectedSet(SPGraph& graph, SPGraph::vertex_descriptor superpixel, std::set<SPGraph::vertex_descriptor>& elements, int depth = 0);

	long double computeProbability();

	static long double computeProposalRatio(MultiviewSegment& SA1, MultiviewSegment& SA2, MultiviewSegment& SB1, MultiviewSegment& SB2);

};


#endif /* MULTIVIEWSEGMENTATION_H_ */
