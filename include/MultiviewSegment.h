/**
 * \file MultiviewSegment.h
 * \brief 
 *
 *  \date Apr 17, 2013
 *  \author arprice
 */

#ifndef MULTIVIEWSEGMENT_H_
#define MULTIVIEWSEGMENT_H_

#include "SPGraph.h"

//#include "GraphUtils.h"

class MultiviewSegment
{
public:
	long segmentID;									///< The segment identifier
	std::set<SPGraph::vertex_descriptor> vertices;	///< The set of super pixels with the same label.
	PlaneModel plane;								///< The information of the previous best plane.
	long hash;										///< Represents the set of superpixels and the type
	long double probability;						///< The probability of the Z_ given S_i, P(Z_i|S_i)

	int r,g,b;

	MultiviewSegment(long id);

	MultiviewSegment(const MultiviewSegment& other);

	void computeFitPlane();

	long double computeProbability();

	//static void setGraph(boost::shared_ptr<SPGraph> graphPtr);

//private:
	static boost::shared_ptr<SPGraph> mGraph;							///< A reference to the world graph.
};



#endif /* MULTIVIEWSEGMENT_H_ */
