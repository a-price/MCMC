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

class MultiviewSegment
{
	std::set<SPGraph::vertex_descriptor> vertices;	///< The set of super pixels with the same label.
	Eigen::Vector4f plane;							///< The information of the previous best plane.
	long hash;										///< Represents the set of superpixels and the type
	long double probabilityL;						///< The probability of the Z_ given S_i, P(Z_i|S_i)
};



#endif /* MULTIVIEWSEGMENT_H_ */
