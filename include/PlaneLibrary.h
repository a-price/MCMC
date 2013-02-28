/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file Segmentation3D.cpp
 * @date October 2nd, 2011
 * @author Can Erdogan
 * @brief The plane info library structure which avoids refitting of surfaces to subgraphs
 */

#pragma once

#include "Graph.h"

#include <stdlib.h>

#include <map>

/// The structure that associates subgraphs with plane fitting results
/// UNIT The PlaneLibrary class
class PlaneLibrary {
public:

	const size_t maxNumSuperPixels_;							///< The max # of pixels a subgraph can have.
	std::map <long, Plane*> map_;						///< From subgraphs hashes to fit surfaceInfo's

	/// The constructor
	PlaneLibrary () : maxNumSuperPixels_(200) {}

	///  Updates the database if the segment has acceptable # of sp's and its surface is computed.
	inline void update (const Segment& segment) {

		// Sanity check
		if(segment.plane_ == NULL) throw Exception("Bad input to PlaneLibrary::update(.)");

		// Check that number of super pixels of the subgraph is within the limit
		if(segment.superPixels_.size() > maxNumSuperPixels_) return;

		// Update the map
		map_.insert(std::make_pair(segment.hash_, segment.plane_));
	}

	/// Returns the PlaneInfo related to the given subgraph
	Plane* getInfo (const Segment& segment) const {
		std::map <long, Plane*>::const_iterator it = map_.find(segment.hash_);
		if(it != map_.end()) return it->second;
		else return NULL;
	}
};
