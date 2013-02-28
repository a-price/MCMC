#include "../include/Segmentation.h"
#include "../include/SurfaceFitting/GlobalFitting.h"

#include <algorithm>
#include <queue>
#include <locale>

using namespace std;

void Segmentation::selectSegment (const Segmentation& segmentation,
    const set <SuperPixel*>& component, Segment* originalSegment,
    const set <Segment*>& neighborSegments, Segment*& nextSegment,
    long double& segmentProposalRatio) {

	// Set the parameters
	static const double newSegmentProb = 0.8;

	// Check if the component is as big as the subgraph it belongs to
	bool componentSegmentSame = (component.size() == originalSegment->superPixels_.size());

	// Decide whether a new subgraph will be chosen. If the component is the original subgraph itself,
	// it does not make sense to create a new subgraph.
	double random = ((double) rand()) / RAND_MAX;
//	printf("\tSelect subgraph random: %f, same: %d\n", random, componentSegmentSame);
	if(!componentSegmentSame && (random < newSegmentProb)) {

		// Create a new subgraph
		nextSegment = new Segment(component);
		segmentProposalRatio = newSegmentProb;
	}

	else {

		// Set the merge probability which would normally be 1 - newSegmentProbability. However,
		// if the component is the subgraph, i.e. no new subgraph option, then mergeProb is 1.0.
		double existingSegmentProb = componentSegmentSame ? (1.0) : (1.0 - newSegmentProb);

		// Decide which subgraph will be chosen (the last one refers to the original subgraph)
		size_t numSegments = neighborSegments.size() + 1;
		size_t randomIndex = rand() % numSegments;

		// Assign to the original subgraph if the randomIndex refers to the last
		if(randomIndex == (numSegments - 1)) {
			nextSegment = originalSegment;
		}

		// Assign it to one of the neighboring subgraphs
		else {
			set <Segment*>::const_iterator segmentIt = neighborSegments.begin();
			advance(segmentIt, randomIndex);
			nextSegment = *segmentIt;
		}

		// Set the probability of choosing a subgraph
		segmentProposalRatio = existingSegmentProb / numSegments;
	}
}

void Segmentation::createComponent (const Segmentation& S, set <SuperPixel*>& component,
    Segment*& originalSegment, set <Segment*>& neighborSegments,
    long double& oldSeparationProbability) {

	static const bool debug = 0;
	if(debug) printf("***********************************************************\n");

	// Pick a superpixel. This is the first superpixel in the component.
	SuperPixel* initialSuperPixel = pickSuperPixel(S);
	component.insert(initialSuperPixel);
	originalSegment = S.assignments_.at(initialSuperPixel);

	// Prepare the queue for the next superPixel to be expanded
	// NOTE Superpixels that have been added to the component constitute as visited. A superpixel
	// might be considered as a neighbor multiple times in case it is not added in the first case.
	queue <SuperPixel*> toBeExpanded;
	toBeExpanded.push(initialSuperPixel);

	// Keep expanding superpixel until the queue is empty.
	// NOTE The neighbor superpixels should be in the same subgraph as the initial superpixel. If they
	// are not, then they belong to a neighboring subgraph that should be recorded as well.
	while (!toBeExpanded.empty()) {

		// Get the next superpixel
		const SuperPixel* superPixel = toBeExpanded.front();
		toBeExpanded.pop();

		// Traverse the neighbors to attempt to add them to the component
		if(debug) printf("Superpixel %lu with %lu neighbors: \n", superPixel->id_, superPixel->neighbors_.size());
		map <SuperPixel*, long double>::const_iterator neighborIt =
		    superPixel->neighbors_.begin();
		for(; neighborIt != superPixel->neighbors_.end(); neighborIt++) {

			// If the neighboring superpixel is already in the component, we can skip it.
			if(debug) printf("\tNeighbor %lu with edge probabilitiy %Lf is ", neighborIt->first->id_, neighborIt->second);
			if(component.count(neighborIt->first) == 1) {
				if(debug) printf("already in the component.\n");
				continue;
			}

			// If the neighboring superpixel is in another segment, we should skip but record it.
			Segment* neighborSegment = S.assignments_.at(neighborIt->first);
			if(neighborSegment != originalSegment) {
				neighborSegments.insert(neighborSegment);
				if(debug) printf("in another segment.\n");
				continue;
			}

			// Flip the edge probability.
			long double edgeProbability = neighborIt->second;
			long double random = ((long double) rand()) / RAND_MAX;
			if(random < edgeProbability) {

				if(debug) printf("added to the component.\n");

				// Add the neighbor to the component and to the queue
				component.insert(neighborIt->first);
				toBeExpanded.push(neighborIt->first);
			}
			else if(debug) printf("rejected.\n");
		}
	}

	// Compute the separation probability by traversing the superpixels in the component and looking
	// for the neighbors that are in the subgraph of the initial superpixel but not in the component.
	// NOTE The reason to do it afterwards is for the simplicity of the code and possible efficiency
	// gain. As we grow, we need to make sure a superpixel not connected with is guaranteed not to be
	// connected by another superpixel again before we accumulate their probability.
	oldSeparationProbability = 1.0;
	set <SuperPixel*>::const_iterator superPixelIt = component.begin();
	for(; superPixelIt != component.end(); superPixelIt++) {

		// Traverse the neighbors of the superpixel
		map <SuperPixel*, long double>::const_iterator neighborIt =
		    (*superPixelIt)->neighbors_.begin();
		for(; neighborIt != (*superPixelIt)->neighbors_.end(); neighborIt++) {

			// If the neighbor is in another subgraph, we skip. (It does not actually separate).
			Segment* neighborSegment = S.assignments_.at(neighborIt->first);
			if(neighborSegment != originalSegment) continue;

			// If the neighbor is not in the component, we accumulate the (1 - edge probability).
			if(component.count(neighborIt->first) == 0) {
				oldSeparationProbability *= (1.0 - neighborIt->second);
			}

		}
	}

	if(debug) fflush(stdout);
}

SuperPixel* Segmentation::pickSuperPixel (const Segmentation& S) {

	// Pick a random number in {0 .. numSuperPixels-1}.
	size_t numSuperPixels = S.assignments_.size();
	size_t randomIndex = (rand() % numSuperPixels);

	// Get the superpixel pointer ...
	map <SuperPixel*, Segment*>::const_iterator superPixelIt = S.assignments_.begin();

	// A. Pick the superpixel with the maximum error
	double random = ((double) rand()) / RAND_MAX;
	if(random < 0.0) {
//		map <long double, const SuperPixel*> errors;
//		for(; superPixelIt != S.assignments_.end(); superPixelIt++) {
//			errors.insert(make_pair(superPixelIt->second->surfaceInfo_->localErrors_->at(superPixelIt->first),
//					superPixelIt->first));
//		}
//		return errors.begin()->second;
//	}

		// B. Pick a superpixel with error more than 1e^5
		for(; superPixelIt != S.assignments_.end(); superPixelIt++) {
//			double error = superPixelIt->second->plane_->localErrors_->at(superPixelIt->first);
//			if(error > 4 * 1e3) return superPixelIt->first;
		}
		fflush(stdout);
	}

	// Continued original ...
	superPixelIt = S.assignments_.begin();
	advance(superPixelIt, randomIndex);
	return superPixelIt->first;
}

long double Segmentation::computeProbability (const Segmentation& segmentation) {

	// Simply multiply the probabilities of the subgraphs
	long double probabilityL = 0.0;
	set <Segment*>::const_iterator segmentIt = segmentation.segments_.begin();
	for(; segmentIt != segmentation.segments_.end(); segmentIt++) {
		probabilityL += (*segmentIt)->probabilityL_;
	}

	return probabilityL;
}

void Segmentation::computeHash (Segmentation& segmentation) {

	// Traverse the superpixels in the order they are in the subgraph. They should not change
	// as the superpixel pointers are not modified after creation.
	map <SuperPixel*, Segment*>::const_iterator segmentIt = segmentation.assignments_.begin();
	map <Segment*, size_t> assignments;
	size_t counter = 0;
	// string buffer;
	char buffer [segmentation.assignments_.size() * 4];
	char temp[256];
	static locale loc;
	static const collate <char>& coll = use_facet <collate <char> >(loc);
	size_t spcounter = 0;
	for(; segmentIt != segmentation.assignments_.end(); segmentIt++) {

		// Check if the segment is already assigned a value
		size_t label;
		map <Segment*, size_t>::iterator previousLabelIt = assignments.find(segmentIt->second);
		if(previousLabelIt == assignments.end()) {

			// Increment the counter
			assignments.insert(make_pair(segmentIt->second, counter));
			label = counter;
			counter++;
		}

		else {
			// Use the previous value
			label = previousLabelIt->second;
		}

		// Save the label
		sprintf(buffer + spcounter * 4, "%3lu,", label);
		// sprintf(temp, "%lu,", label);
		// buffer = buffer + string(temp);
		spcounter++;
	}

	// Make a hash
	segmentation.hash_ = coll.hash(buffer, buffer + strlen(buffer));
}

