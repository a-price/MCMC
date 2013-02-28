/*
 * @file Segmentation.h
 * @date Feb 21, 2011
 * @author Can Erdogan
 * @author Manohar Paluri
 * @brief Contains the Segmentation class that represents a partition of superpixels into segments.
 */

#ifndef _Segmentation_H_
#define _Segmentation_H_

#include "Common.h"
#include "Graph.h"
#include "PlaneLibrary.h"

/// Define the Segmentation
class Segmentation {
public:

	// The constant fields of a state that are created from the input and passed on with constructors

	const Graph& graph_;															///< The underlying graph

	// The variable fields that represent the partitions and the accumulated data (library)

	std::set <Segment*> segments_;									///< The list of segments
	std::map <SuperPixel*, Segment*> assignments_;		///< The assignments of superpixels to segments
	PlaneLibrary* library_;													///< The library of computed plane info's.
	long hash_;																				///< The hash representation of the segmentation

public:

	/// The minimal constructor
	Segmentation (const Graph& graph) :
			graph_(graph), library_(new PlaneLibrary) {
		computeHash(*this);
	}

	/// Creates a new Segmentation using the assignment data structures
	Segmentation (const Graph& graph, const std::set <Segment*>& segments,
	    const std::map <SuperPixel*, Segment*>& assignments, PlaneLibrary* library = NULL) :
			graph_(graph), segments_(segments), assignments_(assignments), library_(
			    library) {
		if(library_ == NULL) library_ = new PlaneLibrary;
		computeHash(*this);
	}

public:
	// The proposal helper functions

	/// Selects a subgraph from the neigbboring subgraphs of the component or the original subgraph,
	/// OR creates a new subgraph. It follows the "merging with the component" section in
	/// implementation.lyx.
	static void selectSegment (const Segmentation& segmentation,
	    const std::set <SuperPixel*>& component, Segment* originalSegment,
	    const std::set <Segment*>& neighborSegments, Segment*& nextSegment,
	    long double& segmentProposalRatio);

	/// Creates a component by selecting a superpixel and growing it following edge probabilities.
	/// Also, computes the separation probability from the original subgraph and determines the
	/// neighboring subgraphs.
	static void createComponent (const Segmentation& segmentation, std::set <SuperPixel*>& component,
	    Segment*& originalSegment, std::set <Segment*>& neighborSegments,
	    long double& oldSeparationProbability);

	/// Picks a superpixel purely randomly.
	static SuperPixel* pickSuperPixel (const Segmentation& segmentation);

	/// Split a component from segment1 and merges it with segment2
	static Segmentation* merge (const Segmentation& segmentation, std::set <SuperPixel*>* component,
	    Segment* oldSegment, Segment* newSegment);

public:

	/// Returns the probability of a segmentation as in Equation 2 in implementation.lyx.
	/// NOTE The prior P(S) is not used/uniform (i.e. it is 1.0).
	/// NOTE This function can be used with a subset of the superpixels assigned to subgraphs as in
	/// the neighbor probability computations.
	/// NOTE The function returns log probabilities.
	static long double computeProbability (const Segmentation& segmentation);

	/// The print function for the Segmentation
	void print (const std::string& s, FILE* stream = stdout) const;

	/// Computes the hash value for the assignments
	static void computeHash (Segmentation& segmentation);

};

#endif /* _Segmentation_H_ */
