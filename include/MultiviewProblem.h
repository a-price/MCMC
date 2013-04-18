/**
 * \file MultiviewProblem.h
 * \brief
 *
 *  \date Apr 3, 2013
 *  \author Andrew Price
 */

#include "MultiviewSegmentation.h"


/// Problem definition for 3D multiview segmentation
class MultiviewProblem
{
public:

	/// The problem's state is a MultiviewSegmentation - a partition of superpixels
	typedef MultiviewSegmentation State;

	/// Create an initial state
	static State* initializeState (void* data = NULL);

	/// Proposes a new state using the SW convention
	static State* propose (const State& state, long double& targetRatio, long double& proposalRatio);

	/// Computes the edge probabilities between the superpixels
	//static void computeEdgeProbs (Graph& graph);



};
