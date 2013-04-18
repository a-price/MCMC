/**
 * \file MultiviewSegmentation.h
 * \brief 
 *
 *  \date Apr 17, 2013
 *  \author arprice
 */

#ifndef MULTIVIEWSEGMENTATION_H_
#define MULTIVIEWSEGMENTATION_H_

#include "SPGraph.h"

class MultiviewSegmentation
{
public:
	const SPGraph& mGraph;


	MultiviewSegmentation (const SPGraph& graph) :
		mGraph(graph)
	{

	}
};


#endif /* MULTIVIEWSEGMENTATION_H_ */
