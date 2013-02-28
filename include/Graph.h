/*
 * @file Graph.h
 * @date Sept 29, 2011
 * @author Can Erdogan
 * @author Manohar Paluri
 * @brief The definition of a graph structure described in Barbu 2003 ICCV or 2005 PAMI.
 */

#pragma once

#include "Common.h"

#include <gtsam/base/timing.h>

#include <limits.h>
#include <map>
#include <set>

class Segmentation;
class PlaneLibrary;

/// The subgraph structure that represents a label and is usually composed of multiple superPixels.
class Segment {
public:

	std::set <SuperPixel*> superPixels_;		///< The set of super pixels with the same label.
	Plane* plane_;										///< The information of the previous best plane.
	long hash_;																		///< Represents the set of superpixels and the type
	long double probabilityL_;											///< The probability of the Z_ given S_i, P(Z_i|S_i)
	cv::Vec3b* color_;

	/// The default constructor
	Segment (const std::set <SuperPixel*>& superPixels, 
	    PlaneLibrary* library = NULL) :
			superPixels_(superPixels), color_(NULL), plane_(NULL) {
		gttic_(Compute_hash);
		computeHash(*this);
		gttoc_(Compute_hash);
		computeProbability(*this, library);
	}

	/// The print function
	void print (const std::string& s, FILE* stream = stdout) const;

	/// Writes the contents of the subgraph into a single pcd file. Designed to visualize subgraphs
	/// together by using pcd_viewer on multiple pcd files.
	void writeToPCD (const std::string& s) const;

public:

	// The helper functions for the proposal: 1) the combine and remove constructors and 2) edge cut
	// set probability of a set with a subgraph.

	/// Combines the subgraph's superpixels with the input superpixels and creates a new subgraph.
	static Segment* combine (const Segment& subGraph, const std::set <SuperPixel*>& toAdd);

	/// Removes the input superpixels from the given subgraph and creates a new subgraph.
	static Segment* diff (const Segment& subGraph, const std::set <SuperPixel*>& toRemove);

	/// Computes the product of the 1 - probabilities of the edges in the cut set of the subgraph and
	/// the given set.
	static double mergeProbability (const Segment& subGraph,
	    const std::set <SuperPixel*>& superPixels,
	    const std::map <SuperPixel*, Segment*>& assignments);


private:

	// The helper functions to compute the global plane and the associated probability.

	/// Fits a surface to the set of superpixels of this subgraph
	static Plane* fitGlobalSurface (Segment& subGraph, 
	    PlaneLibrary* library = NULL);

	/// Computes the probability of a subgraph P(Z_i|S_i) using the integral in equation 2 in
	/// implementation.lyx.
	static void computeProbability (Segment& subGraph, PlaneLibrary* library =
	    NULL);

	/// Computes a hash value for the subgraph using the superpixels and the type
	static void computeHash (Segment& subGraph);

};

/// Holds the informations about the superpixels
struct Graph {
	std::map <SuperPixelID, SuperPixel*> superPixels_;				///< The superpixels
	std::set <size_t> boundaryPixels_;												///< The boundary pixels around superpixels
};

