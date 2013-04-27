/**
 * \file PatternedMatcher.cpp
 * \brief 
 *
 *  \date Apr 19, 2013
 *  \author Andrew Price
 */

#include "GraphUtils.h"
#include <Eigen/Core>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>

#include "SPGraph.h"
#include "Serialization.h"
#include "MultiviewProblem.h"

const int NUM_ITERATIONS = 10;

int main(int argc, char** argv)
{
	srand(time(NULL));

	boost::shared_ptr<SPGraph> gPtr(new SPGraph);
	SPGraph& spGraph = *(gPtr);

	std::ifstream ifs("test.big", std::fstream::in | std::fstream::binary );
	{
		boost::archive::binary_iarchive ia(ifs);
		// read class instance from archive
		ia >> spGraph;
		// archive and stream closed when destructors are called
	}

	// Initialize a random partition
	MultiviewSegment::mGraph = gPtr;
	MultiviewSegmentation segmentation(spGraph);
	boost::mt19937 gen(time(0));

	for (int mcmcStep = 0; mcmcStep < NUM_ITERATIONS; mcmcStep++)
	{
		// Select a (soon-to-be) weighted random superpixel
		SPGraph::vertex_descriptor start = boost::random_vertex(spGraph, gen);

		// Get the parent segment
		MultiviewSegment* originalSegment = (segmentation.getParentSegment(start));
		MultiviewSegment* targetSegment;
		MultiviewSegment* originalSegmentB;
		MultiviewSegment* targetSegmentB;

		std::cerr << "\nSegment info:\tID: " << originalSegment->segmentID << "\tVertices: " << originalSegment->vertices.size() << std::endl;

		// Grow the superpixel into a connected component
		std::set<SPGraph::vertex_descriptor> elements;
		segmentation.getNewConnectedSet(spGraph, start, elements);

		// Select type of move to propose: Split or Transfer
		double moveType = MathUtils::randbetween(0,1);
		if (moveType < 0.10)
		{
			std::cerr << "Splitting segment..." << std::endl;
			// Split into a new segment
			targetSegment = segmentation.addNewSegment(elements);
		}
		else
		{
			std::cerr << "Picking neighbor...";
			// Pick a neighboring segment
			std::set<MultiviewSegment*> neighbors = segmentation.getNeighborSegments(elements);
			if (neighbors.size() < 1)
				{ continue; }
			int neighborIdx = round(MathUtils::randbetween(0,neighbors.size()-1));
			//std::cerr << neighbors.size();
			std::set<MultiviewSegment*>::iterator iter = neighbors.begin();
			std::advance(iter, neighborIdx);

			targetSegment = (*iter);
		}


		// Add selected superpixels to that segment
		std::cerr << "Moving segment..." << std::endl;
		originalSegmentB = (new MultiviewSegment(*originalSegment));
		targetSegmentB = (new MultiviewSegment(*targetSegment));
		segmentation.moveSuperpixels(elements, *originalSegment, *targetSegment, *originalSegmentB, *targetSegmentB);

		// Compute M-H proposal ratio and accept move probabilistically
		long double a = MultiviewSegmentation::computeProposalRatio(*originalSegment, *targetSegment, *originalSegmentB, *targetSegmentB);
		bool proposalAccepted = MathUtils::randbetween(0,1) < a;

		if (proposalAccepted)
		{
			std::cerr << "\nMove Accepted.\n";
		}
		else
		{
			std::cerr << "\nMove Rejected.\n";
		}


		// Record state and score
	}

}
