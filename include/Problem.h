/*
 * @file Problem.h
 * @date Sept 29, 2011
 * @author Can Erdogan
 * @author Manohar Paluri
 * @brief The problem definition for the Planar Segmentation problem defined in crv2012 submission
 * under docs.
 */

#ifndef _PROBLEM_H_
#define _PROBLEM_H_

#include "Segmentation.h"

/// The problem definition of 3D segmentation
class Problem {
public:

	/// The state of the problem is a segmentation - a partition of superpixels into segments
	typedef Segmentation State;

	/// Create an initial state
	static State* initializeState (void* data = NULL);

	/// Proposes a new state using the SW convention
	static State* propose (const State& state, long double& targetRatio, long double& proposalRatio);

	/// Computes the edge probabilities between the superpixels
  static void computeEdgeProbs (Graph& graph);

public:
	// These functions are for visualizing a segmentation.

	/// The handler for mouse events for the visualized opencv image
	static void interact(int event, int x, int y, int flags, void *param);

	/// Creates an image out of a segmentation
	static cv::Mat* visualize(const Segmentation& S, const std::set <SuperPixel*>* component = NULL, 
			bool drawError = false);

private:

	/// Computes the jump probability from the segments {S^A_1, S^A_2} to {S^B_1, S^B_2} using
	/// equation 3 in implementation.lyx.
	/// NOTE sA2 and sB2 are optional because: 1 -> 2 (split), 2 -> 2 (exchange), 2 -> 1 (merge).
	static long double jumpProbability (const Segment& sA1, const Segment& sB2,
	    const Segment* sA2 = NULL, const Segment* sB1 = NULL);

	/// Creates a new segmentation by changing the segments {S^A_1, S^A_2} to {S^B_1, S^B_2}. The
	/// component leave S^A_1 to merge with S^B_2. If S^A_2 is NULL, S^B_2 is a new subgraph (split);
	/// otherwise, an exchange happened between S^A_1 and S^A_2. If S^B_1 is NULL, that means the
	/// component was the original subgraph and it is not needed anymore. Otherwise, it is the rest
	/// of the original subgraph after component is removed.
	/// NOTE The assignments and the subgraphs should be moved carefully from A to new segmentation B.
	/// NOTE Assumes that the new subgraphs sB1 and sB2 two are created and allocated in the heap.
	static Segmentation* jump (const Segmentation& A, Segment& sA1, Segment& sB2, Segment* sA2 =
	    NULL, Segment* sB1 = NULL);

private:

	// Edge probability computations
	// NOTE The following 3 functions need to be here because the edge probability depends on
	// Segmentation target probabilities and the Graph is a constant in the Segmentation class.

	/// Creates two segmentations S1 and S2 out of the two plane data sets where S1 has both
	/// superpixels in the same segment and S2 has them separate.
	static void createSegmentations (const Graph& graph,
	    SuperPixel& superPixel1, SuperPixel& superPixel2, Segmentation*& together,
	    Segmentation*& separate);

	/// Computes the edge probability for two super pixels
	static double computeEdgeProbability (const Graph& graph,
	    SuperPixel& sp1, SuperPixel& sp2);

public:

	// The friend class definitions for testing purposes
	// TODO Find a clever macro to replace these tests.

	friend class testEdgeProbabilitiesTest;
};

#endif // _PROBLEM_H_
