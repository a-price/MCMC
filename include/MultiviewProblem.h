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
	static State* initializeState (void* data = NULL)
	{
		if(data == NULL) throw Exception("Initialize state needs a graph pointer");
		SPGraph* graph = reinterpret_cast <SPGraph*> (data);

		// Note that MultiviewSegmentation creates its own initial connected sets.
		State* state = new State(*graph);

		return state;
	}

	/// Proposes a new state using the SW convention
	static State* propose (const State& state, long double& targetRatio, long double& proposalRatio)
	{
		// ============================================================================================
		// 1. Create the component.

		// ============================================================================================
		// 2. Select the subgraph to combine with

		// ============================================================================================
		// 3. Compute the proposal ratio based on 3 cases (and return if same state is proposed)

		// ============================================================================================
		// 4. Compute the target ratio and assign the component to the chosen subgraph
	}



};
