/**
 * \file MultiviewSegmentation.h
 * \brief 
 *
 *  \date Apr 17, 2013
 *  \author Andrew Price
 */

#ifndef MULTIVIEWSEGMENTATION_H_
#define MULTIVIEWSEGMENTATION_H_

#include "Graph.h" // for some reason needed for gtsam...
#include "MultiviewSegment.h"

class MultiviewSegmentation
{
public:
	SPGraph& mGraph;
	//const SPFilteredGraph& mFGraph;

	std::vector<MultiviewSegment> segments;
	//std::map<long, MultiviewSegment> segments;

	MultiviewSegmentation (SPGraph& graph);

	MultiviewSegment* getParentSegment(SPGraph::vertex_descriptor target);

	MultiviewSegment* addNewSegment(std::set<SPGraph::vertex_descriptor> elements);

	std::set<MultiviewSegment*> getNeighborSegments(std::set<SPGraph::vertex_descriptor> elements);

	void moveSuperpixels(std::set<SPGraph::vertex_descriptor> elements, MultiviewSegment* targetSegment);
};


#endif /* MULTIVIEWSEGMENTATION_H_ */
