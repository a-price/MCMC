/**
 * \file MultiviewSegmentation.cpp
 * \brief 
 *
 *  \date Apr 20, 2013
 *  \author Andrew Price
 */

#include "MultiviewSegmentation.h"

MultiviewSegmentation::MultiviewSegmentation(SPGraph& graph) :
		mGraph(graph)
{
	// Loop through all edges and probabilistically turn them on
	isProposal = STATE_TYPE::ACCEPTED;
	int count = 0;
	probability = 0;
	SPGraph::edge_iterator edgeIt, edgeEnd;
	boost::tie(edgeIt, edgeEnd) = boost::edges(graph);
	for (; edgeIt != edgeEnd; ++edgeIt)
	{
		SPEdge & edge = graph[*edgeIt];
		edge.currentState->partitionOn = (MathUtils::randbetween(0, 1) <= (edge.BernoulliProbability));
		if (edge.currentState->partitionOn)
		{
			count++;
		}
	}
	// Get a filtered graph to do connected components on
	SPFilteredGraph fGraph(graph, SPEdgePredicate(graph));

	// Get the connected sets and return them
	std::map<SPGraph::vertex_descriptor, SPGraph::vertices_size_type> components;
	boost::associative_property_map<
			std::map<SPGraph::vertex_descriptor, SPGraph::vertices_size_type>> componentsMap(components);
	int numComponents = connected_components(fGraph, componentsMap);

	// Create Segments based on connected components
	for (int i = 0; i < numComponents; i++)
	{
		MultiviewSegment segment(i);
		segment.segmentID = i;
		segments.push_back(segment);
	}

	// Add superpixels to segments
	for (int i = 0; i < components.size(); i++)
	{
		std::map<SPGraph::vertex_descriptor, SPGraph::vertices_size_type>::iterator component =	components.find(i);
		if (component == components.end())
		{
			// Problem, this should not happen
			std::cerr << "Missing vertex.\n";
			continue;
		}
		segments[component->second].vertices.insert(component->first);

		// Set the segment ID for each superpixel
		graph[component->first].currentState->currentSegmentID = component->second;
	}

//	std::cout << std::endl << numComponents << " segments of sizes: ";
	for (int i = 0; i < segments.size(); i++)
	{
		segments[i].computeFitPlane();
	}
//	std::cout << std::endl;
}

MultiviewSegment* MultiviewSegmentation::getParentSegment(
		SPGraph::vertex_descriptor target)
{
	for (int i = 0; i < segments.size(); i++)
	{
		if (segments[i].vertices.find(target) != segments[i].vertices.end())
		{
			return &(segments[i]);
		}
	}
	// TODO: Throw Error, since pixel not found...
	return NULL;
}

MultiviewSegment* MultiviewSegmentation::addNewSegment(std::set<SPGraph::vertex_descriptor> elements)
{
	// Create the new segment
	MultiviewSegment* newSegment = (new MultiviewSegment(segments.size()));
	//newSegment.segmentID = segments.size();

	for (std::set<SPGraph::vertex_descriptor>::iterator i = elements.begin(); i != elements.end(); i++)
	{
		// Remove superpixels from other segments
		MultiviewSegment* oldSegment = getParentSegment(*i);
		oldSegment->vertices.erase(*i);

		// Add superpixel to new component
		newSegment->vertices.insert(*i);

		// Change parent segment in vertices
		mGraph[*i].proposedState->currentSegmentID = newSegment->segmentID;
	}

	newSegment->computeFitPlane();

	segments.push_back(*newSegment);

	return &segments[segments.size()-1];
}

std::set<MultiviewSegment*> MultiviewSegmentation::getNeighborSegments(std::set<SPGraph::vertex_descriptor> elements)
{
	std::set<MultiviewSegment*> neighbors;

	for (std::set<SPGraph::vertex_descriptor>::iterator i = elements.begin(); i != elements.end(); i++)
	{
		// Loop through all neighbor vertices
		SPGraph::out_edge_iterator outEdgeIt, outEdgeEnd;
		boost::tie(outEdgeIt, outEdgeEnd) = boost::out_edges(*i, mGraph);
		for (; outEdgeIt != outEdgeEnd; ++outEdgeIt)
		{
			// Get a neighbor superpixel
			SPGraph::vertex_descriptor neighborID = boost::target(*outEdgeIt, mGraph);

			// If it's in a different segment, add that segment to the neighbor set
			if (mGraph[neighborID].currentState->currentSegmentID != mGraph[*i].currentState->currentSegmentID)
			{
				for (int i = 0; i < segments.size(); i++)
				{
					if (segments[i].segmentID == mGraph[neighborID].currentState->currentSegmentID)
					{
						neighbors.insert(&segments[i]);
					}
				}

			}
		}
	}

	return neighbors;
}

void MultiviewSegmentation::moveSuperpixels(std::set<SPGraph::vertex_descriptor> elements, MultiviewSegment& SA1, MultiviewSegment& SA2, MultiviewSegment& SB1, MultiviewSegment& SB2)
{
	for (std::set<SPGraph::vertex_descriptor>::iterator i = elements.begin(); i != elements.end(); i++)
	{
		// Remove superpixels from other segments
		SB1.vertices.erase(*i);

		// Add superpixel to new component
		SB2.vertices.insert(*i);

		// Change parent segment in vertices
		SPNode& vertex = mGraph[*i];
		vertex.proposedState->currentSegmentID = SB2.segmentID;
	}

	SB1.computeFitPlane();
	SB2.computeFitPlane();
}

void MultiviewSegmentation::getNewConnectedSet(SPGraph& graph, SPGraph::vertex_descriptor superpixel, std::set<SPGraph::vertex_descriptor>& elements, int depth)
{
	// Make sure we add descriptor to set of nodes
	elements.insert(superpixel);
	++depth;

	// Loop through all neighbor vertices
	SPGraph::out_edge_iterator outEdgeIt, outEdgeEnd;
	boost::tie(outEdgeIt, outEdgeEnd) = boost::out_edges(superpixel, graph);
	for (; outEdgeIt != outEdgeEnd; ++outEdgeIt)
	{
		// Get a neighbor superpixel
		SPGraph::vertex_descriptor neighborID = boost::target(*outEdgeIt, graph);
		//for (int i = 0; i < depth; i++) {std::cout << "\t";}
		//std::cout << superpixel;

		// See if it's in the same current segment
		if (graph[superpixel].currentState->currentSegmentID != graph[neighborID].currentState->currentSegmentID)
		{
			continue;
		}

		// See if it's already in our set
		if (elements.find(neighborID) != elements.end())
		{
			// The vertex is already one of our connected components, so keep iterating
			//std::cout << " --O " << neighborID << std::endl;
			continue;
		}

		// Compute a random variable for the jump
		SPEdge & edge = graph[*outEdgeIt];
		edge.proposedState->partitionOn = (MathUtils::randbetween(0,1) <= (edge.BernoulliProbability));

		// If connected, recurse into the new node and keep growing
		if (edge.proposedState->partitionOn)
		{
			//std::cout << " --> " << neighborID << std::endl;

			// Continue depth-first search at new node
			getNewConnectedSet(graph, neighborID, elements, depth);
		}
		else
		{
			//std::cout << " --X " << neighborID << std::endl;
		}

	}

	--depth;
}

long double MultiviewSegmentation::computeProbability()
{
	probability = 1;
	for (int i = 0; i < segments.size(); i++)
	{
		probability *= segments[i].computeProbability();
	}

	return probability;
}

long double MultiviewSegmentation::computeProposalRatio(MultiviewSegment& SA1, MultiviewSegment& SA2, MultiviewSegment& SB1, MultiviewSegment& SB2)
{
	// TODO: compute edge probabilities as well
	return (SB1.computeProbability()/SA1.computeProbability())*(SB2.computeProbability()/SA2.computeProbability());
}
