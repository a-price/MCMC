/**
 * \file MultiviewSegmentation.h
 * \brief 
 *
 *  \date Apr 17, 2013
 *  \author arprice
 */

#ifndef MULTIVIEWSEGMENTATION_H_
#define MULTIVIEWSEGMENTATION_H_

#include "MultiviewSegment.h"

class MultiviewSegmentation
{
public:
	const SPGraph& mGraph;
	const SPFilteredGraph& mFGraph;

	std::vector<MultiviewSegment> segments;

	MultiviewSegmentation (SPGraph& graph) :
		mGraph(graph), mFGraph(getNewPartition(graph))
	{
		// Get the connected sets
		std::vector<int> component(num_vertices(mGraph));
		int num = connected_components(mFGraph, &component[0]);

		std::vector<SPGraph::vertex_descriptor>::size_type i;

		// add each
		segments(num);
		for (i = 0; i < component.size(); i++)
		{
			MultiviewSegment& segment = segments[component[i]];
			segment.vertices.insert(i);
		}
	}
};


#endif /* MULTIVIEWSEGMENTATION_H_ */
