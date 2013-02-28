/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file Problem.cpp
 * @date October 2nd, 2011
 * @author Can Erdogan
 * @author Manohar Paluri
 * @brief The problem definition following the graph structure described in Barbu 2003 ICCV or 2005 PAMI.
 */

#include "../include/IO.h"
#include "../include/Problem.h"

#include <gtsam/base/timing.h>
#include <gtsam/linear/GaussianDensity.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace gtsam;

static const bool debugTiming = true;

Segmentation* Problem::jump (const Segmentation& A, Segment& sA1, Segment& sB2, Segment* sA2,
    Segment* sB1) {

	// Create the new segmentation B as a copy of the old one, A.
	Segmentation* B = new Segmentation(A.graph_, A.segments_, A.assignments_, A.library_);

	// ===========================================================================
	// Updates to the assignment of superpixels

	// 1. If the original subgraph still persists after the split, its assignments should be updated.
	if(sB1 != NULL) {
		set <SuperPixel*>::const_iterator superPixelIt = sB1->superPixels_.begin();
		for(; superPixelIt != sB1->superPixels_.end(); superPixelIt++)
			B->assignments_[*superPixelIt] = sB1;
	}

	// 2. The component superpixels should be assigned to their new subgraph.
	set <SuperPixel*>::const_iterator superPixelIt = sB2.superPixels_.begin();
	for(; superPixelIt != sB2.superPixels_.end(); superPixelIt++)
		B->assignments_[*superPixelIt] = &sB2;

	// ===========================================================================
	// Updates to the set of subgraphs

	// 1. The old subgraphs should be removed.
	if(1 != B->segments_.erase(&sA1)) throw Exception("Erase sA1 problem.");
	if((sA2 != NULL) && (1 != B->segments_.erase(sA2))) throw Exception("Erase sA1 problem.");

	// 2. The new subgraphs should be added.
	pair <set <Segment*>::iterator, bool> result = B->segments_.insert(&sB2);
	if(!result.second) throw Exception("Insert sB2 problem.");
	if(sB1 != NULL) {
		result = B->segments_.insert(sB1);
		if(!result.second) throw Exception("Insert sB1 problem.");
	}

	// ===========================================================================
	// Important: The hash of the subgraph should be computed again.

	Segmentation::computeHash(*B);

	return B;
}

void Problem::computeEdgeProbs(Graph& graph) {
  
  static const bool debug = false;
  if(debug) printf("COMPUTING EDGE PROBABILITIES:\n");

  gttic_(Edge_probabilities);
  // Iterate through the superpixels
  map <SuperPixelID, SuperPixel*>::iterator superPixelIt = graph.superPixels_.begin();
  for(; superPixelIt != graph.superPixels_.end(); superPixelIt++) {

    // Get the superPixel pointer
    SuperPixel* superPixel = superPixelIt->second;

    // Traverse the neighbors of each superpixel
    map <SuperPixel*, long double>::iterator neighborIt = superPixel->neighbors_.begin();
    for(; neighborIt != superPixel->neighbors_.end(); neighborIt++) {

      // Get handles
      SuperPixel* neighbor = const_cast <SuperPixel*>(neighborIt->first);
      long double edgeProbability = neighborIt->second;

      // Check if this edge has already been computed
      if(edgeProbability != -1.0) continue;

      // Compute the edge probability and update both super pixels
      edgeProbability = computeEdgeProbability(graph, *superPixel, *neighbor);
      neighborIt->second = edgeProbability;
      neighbor->neighbors_[superPixel] = edgeProbability;

      if(debug) {
        printf("%lu (%Lf), ", neighbor->id_, neighborIt->second);
        fflush(stdout);
      }   
    }   
    if(debug) printf("\n");
  }
  gttoc_(Edge_probabilities);

}

Problem::State* Problem::propose (const State& state, long double& targetRatio,
    long double& proposalRatio) {

	static const bool debug = false;
	static const bool visualization = 0;

	// ============================================================================================
	// 1. Create the component.

	set <SuperPixel*>* component = new set <SuperPixel*>();
	Segment* originalSegment;
	set <Segment*> neighborSegments;
	long double oldSeparationProbability;
	Segmentation::createComponent(state, *component, originalSegment, neighborSegments,
	    oldSeparationProbability);

	if(debug) {
		printf("\n\ncomponent size: %lu\n", component->size());
		printf("original subgraph size: %lu\n",
		    state.assignments_.at(*(component->begin()))->superPixels_.size());
		printf("neighborSegments size: %lu\n", neighborSegments.size());
		printf("oldSeparationProbability: %Le\n", oldSeparationProbability);
	}

	if(visualization) {
    cvNamedWindow("image", CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback("image", interact, (void*) (&state));
		cv::Mat* image = visualize(state, component);
		imshow("image", *image);
		cv::waitKey(0);
	}

	// ============================================================================================
	// 2. Select the subgraph to combine with

	Segment* nextSegment;
	long double segmentProposalRatio;
	Segmentation::selectSegment(state, *component, originalSegment, neighborSegments, nextSegment,
	    segmentProposalRatio);
	bool componentSegmentSame = (component->size() == originalSegment->superPixels_.size());

	// ============================================================================================
	// 3. Compute the proposal ratio based on 3 cases (and return if same state is proposed)

	Segment* sA1, *sA2, *sB1, *sB2;
	if(originalSegment == nextSegment) {								// Case A: Merged back to its subgraph

		if(debug) printf("\tMerge back to its subgraph\n");

		// Set the proposal and target ratios
		proposalRatio = segmentProposalRatio;						// Because edge probabilities cancel
		targetRatio = 1.0;																// Because the two segmentations are the same

		// Create a new instance of the current state and return it
		return new Segmentation(state.graph_, state.segments_, state.assignments_);
	}

	else if(state.segments_.count(nextSegment) == 0) {	// Case B: New subgraph (i.e. split)

		if(debug) printf("\tNew subgraph (i.e. split)\n");

		// Set the subgraphs for target ratio computation. A refers to current segmentation, B next.
		sA1 = originalSegment;
		sA2 = NULL;
		sB2 = new Segment(*component);

		// NOTE The component is guaranteed to be a subset of the original subgraph by selectSegment.
		if(component->size() >= originalSegment->superPixels_.size()) throw Exception("Subset problem.");
		sB1 = Segment::diff(*originalSegment, *component);

		// Set the proposal ratio. The merge probability is 1.0 because the component is merging with
		// itself and thus, the cut set is empty.
		proposalRatio = segmentProposalRatio * (1.0 / oldSeparationProbability);
	}

	else {																								// Case C: Comp. exchange or subgraph merge

		if(debug) printf("\t%s\n", componentSegmentSame ? "Subgraph merge" : "Component exchange");

		// Set the subgraphs for target ratio computation. A refers to current segmentation, B next.
		sA1 = originalSegment;
		sA2 = nextSegment;
		sB2 = Segment::combine(*nextSegment, *component);

		// Remove the component from the original subgraph if the component is not the subgraph.
		sB1 = componentSegmentSame ? NULL : Segment::diff(*originalSegment, *component);

		// Compute the merge prob. of the component with the chosen subgraph for the proposal ratio.
		long double newMergeProb = Segment::mergeProbability(*nextSegment, *component,
		    state.assignments_);
		proposalRatio = segmentProposalRatio * (newMergeProb / oldSeparationProbability);
	}

	// ============================================================================================
	// 4. Compute the target ratio and assign the component to the chosen subgraph

	// Compute the target ratio
	targetRatio = jumpProbability(*sA1, *sB2, sA2, sB1);

	// Sanity check for the outputs
	if(isnan(targetRatio)) throw Exception("Target ratio is nan!");
	if(isnan(proposalRatio)) throw Exception("Proposal ratio is nan!");

	// Create and return the new segmentation
	Segmentation* next = jump(state, *sA1, *sB2, sA2, sB1);

	return next;

}

void Problem::interact(int event, int x, int y, int flags, void *param) {
  
	// Get the superpixel if a button is down
	static SuperPixel* lastSP;
	SuperPixel* sp = NULL;
	const Segmentation* S = (const Segmentation*) param;
	if((event == CV_EVENT_LBUTTONDOWN) || (event == CV_EVENT_RBUTTONDOWN)) {

		// Traverse all the superpixels
		map <SuperPixel*, Segment*>::const_iterator assIt = S->assignments_.begin();
		for(; assIt != S->assignments_.end(); assIt++) {
			
			// Traverse all the pixels
			for(size_t row_idx = 0; row_idx < assIt->first->A_->rows(); row_idx++) {
				size_t u = (size_t) (*assIt->first->A_)(row_idx, 0);
				size_t v = (size_t) (*assIt->first->A_)(row_idx, 1);
				if((u == x) && (v == y)) {
					sp = assIt->first;
					break;
				}
			}
		}

		// Return if could not find a superpixel
		if(sp == NULL) return;
	}

	// Print local information about the superpixel if it is chosen with left button
	if(event == CV_EVENT_LBUTTONDOWN) {

		printf("Chose superpixel %lu\n", sp->id_);
		printf("  Number of neighbors: %lu\n", sp->neighbors_.size());
    map <SuperPixel*, long double>::iterator neighborIt = sp->neighbors_.begin();
    //for(; neighborIt != sp->neighbors_.end(); neighborIt++) 
		//	printf("    id: %lu, edge probability: %Lf\n", neighborIt->first->id_, neighborIt->second);
		Vector local_ = sp->plane_->density_.mean();
		Vector local = disparityToExplicit(sp->plane_->density_.mean());
		printf("  Local plane (xyz): <%.3lf, %.3lf, %.3lf>, error: %.3lf\n", local(0), local(1), local(2), sp->plane_->error_);
		printf("  Local plane (theta): <%.3lf, %.3lf, %.3lf>, error: %.3lf\n", local_(0), local_(1), local_(2), sp->plane_->error_);
		cout << "Covariance: \n" << sp->plane_->density_.covariance() << endl;
		const Segment* seg = S->assignments_.at(sp);
		Vector global = disparityToExplicit(seg->plane_->density_.mean());
		printf("  Segment plane (xyz): <%.3lf, %.3lf, %.3lf>, error: %.3lf\n", global(0), global(1), global(2), seg->plane_->error_);

		lastSP = sp;
	}	
	
	else if ((event == CV_EVENT_RBUTTONDOWN) && (lastSP != NULL)) {

		printf("\n\n\n\n\n");
		printf("The edge between %lu and %lu: %Lf\n", sp->id_, lastSP->id_, sp->neighbors_.at(lastSP));
		cout << "  Local plane 1 (theta): " << sp->plane_->density_.mean().transpose() << endl;
		cout << "  Local plane 2 (theta): " << lastSP->plane_->density_.mean().transpose() << endl;
		computeEdgeProbability(S->graph_, *sp, *lastSP);

		// Write the A and b matrices of each superpixel into files
		ofstream ifs ("A1");
		ifs << *(sp->A_);	
		ifs.close();
		ofstream ifs2 ("A2");
		ifs2 << *(lastSP->A_);	
		ifs2.close();
		ofstream ifs3 ("b1");
		ifs3 << *(sp->b_);	
		ifs3.close();
		ofstream ifs4 ("b2");
		ifs4 << *(lastSP->b_);	
		ifs4.close();
		ofstream ifs5 ("p1");
		ifs5 << (sp->plane_->density_.mean());	
		ifs5.close();
		ofstream ifs6 ("p2");
		ifs6 << (lastSP->plane_->density_.mean());
		ifs6.close();
		lastSP = NULL;
	}
		
}

cv::Mat* Problem::visualize(const Segmentation& S, const std::set <SuperPixel*>* component, bool drawError) {

	if(component != NULL) printf("component size: %lu\n", component->size());

	// Initialize the image with white
	cv::Mat* image = new cv::Mat (kHeight, kWidth, CV_8UC3);
	for (int y = 0; y < kHeight; y++) 
		for (int x = 0; x < kWidth; x++) 
			image->at<cv::Vec3b>(y,x) = cv::Vec3b(255, 255, 255);

	// Traverse the pixels to draw their error form or segment color
	map <SuperPixel*, Segment*>::const_iterator assIt = S.assignments_.begin();
	for(; assIt != S.assignments_.end(); assIt++) {

		// Set the error color if requested
		cv::Vec3b color;
		if(drawError) {

			// Compute the Euclidean plane coordinates for both surfaces
			Vector explicitSegment = disparityToExplicit(assIt->second->plane_->density_.mean());
			Vector normalSegment = Vector_(3, explicitSegment(0), explicitSegment(1), explicitSegment(2));
			Vector explicitSuperPixel = disparityToExplicit(assIt->first->plane_->density_.mean());
			Vector normalSuperPixel = Vector_(3, explicitSuperPixel(0), explicitSuperPixel(1),
					explicitSuperPixel(2));

			// Find the angle between the normals of the surfaces
			double dotProd = normalSegment.dot(normalSuperPixel);
			double angle = acos(dotProd);
			size_t errorColor = (size_t) 255 * (1 - (angle / (M_PI / 2.0)));
			color = cv::Vec3b(errorColor, errorColor, 255);

		}

		// Set the random color - reuse if one exists 
		else {

			// Check if the segment already has a color; if not create one
			if(assIt->second->color_ == NULL) 
				assIt->second->color_ = new cv::Vec3b(rand() % 256, rand() % 256, rand() % 256);
			color = *(assIt->second->color_);
		}
		
		// Overwrite the color if the superpixel is in a component
		if((component != NULL) && (component->count(assIt->first) != 0)) {
			color = (drawError ? cv::Vec3b(125, 0, 125) : cv::Vec3b(0, 0, 0));
		}

		// Draw the superpixel
		for(size_t row_idx = 0; row_idx < assIt->first->A_->rows(); row_idx++) {
			size_t u = (size_t) (*assIt->first->A_)(row_idx, 0);
			size_t v = (size_t) (*assIt->first->A_)(row_idx, 1);
			image->at<cv::Vec3b>(v,u) = color;
		}

	}

	// Set the boundary pixels
	set <size_t>::const_iterator pixelIt = S.graph_.boundaryPixels_.begin();
	for(; pixelIt != S.graph_.boundaryPixels_.end(); pixelIt++) {
		size_t pixelIdx = *pixelIt;
		size_t u = pixelIdx % kWidth, v = pixelIdx / kWidth;
		image->at<cv::Vec3b>(v,u) = cv::Vec3b(0,0,0);
	}

	return image;
}

void Problem::createSegmentations (const Graph& graph, 
    SuperPixel& superPixel1, SuperPixel& superPixel2, Segmentation*& together,
    Segmentation*& separate) {

	// 1. Make the first segmentation with only one subgraph

	// Create the single subgraph
	set <SuperPixel*> superPixels;
	superPixels.insert(&superPixel1);
	superPixels.insert(&superPixel2);
	Segment* segment1 = new Segment(superPixels);

	// Create the segmentation arguments
	map <SuperPixel*, Segment*> assignments;
	assignments[&superPixel1] = segment1;
	assignments[&superPixel2] = segment1;

	set <Segment*> segments;
	segments.insert(segment1);

	// Create the segmentation
	together = new Segmentation(graph, segments, assignments);

	// 2. Make the second segmentation with two subgraphs, one for each superpixel.

	// Create the first subgraph
	superPixels.clear();
	superPixels.insert(&superPixel1);
	Segment* segment2 = new Segment(superPixels);

	// Create the second subgraph
	superPixels.clear();
	superPixels.insert(&superPixel2);
	Segment* segment3 = new Segment(superPixels);

	// Create the segmentation arguments
	assignments.clear();
	assignments[&superPixel1] = segment2;
	assignments[&superPixel2] = segment3;

	segments.clear();
	segments.insert(segment2);
	segments.insert(segment3);

	// Create the segmentation
	separate = new Segmentation(graph, segments, assignments);
}

double Problem::computeEdgeProbability (const Graph& graph, 
    SuperPixel& sp1, SuperPixel& sp2) {

	static const bool debug = 0;

	// Create the two segmentations for merged and separate cases
	Segmentation* together, *separate;
	createSegmentations(graph, sp1, sp2, together, separate);

	// Compute the ratio of the probabilities
	// NOTE Even though the probabilities are in log, it is save to use the ratio in base 10 because
	// it is guaranteed to be less than 0.
	// TODO This sounds dumb. Why do we compute the prob. of an entire segmentation?
	if(debug) cout << "\n\nComputing together!\n";
	long double probTogetherL = Segmentation::computeProbability(*together);
	if(debug) cout << "\n\nComputing separate!\n";
	long double probSeparateL = Segmentation::computeProbability(*separate);
	long double ratioL = probTogetherL - addLogProbs(probTogetherL, probSeparateL);
	long double ratio = exp(ratioL);
	if(debug) printf("(%lu vs. %lu) => probTogetherL: %Lf, probSeparateL %Lf, ratioL: %Lf, ratio: %Lf\n", 
			sp1.id_, sp2.id_, probTogetherL, probSeparateL, ratioL, ratio);

	// Sanity check
	if(isnan(ratio)) throw Exception("The edge probability is nan!");
	return ratio;
}

Problem::State* Problem::initializeState (void* data) {

	// A. Initialize a segmentation: get the initial hash, create the library and save the graph
	if(data == NULL) throw Exception("Initialize state needs a graph pointer");
	Graph* graph = reinterpret_cast <Graph*> (data); 
	State* state = new State(*graph);

	// B. Place all the superpixels in a single segment

	// Get the superpixels from the graph
	set <SuperPixel*> superPixels;
	map <SuperPixelID, SuperPixel*>::const_iterator superPixelIt = graph->superPixels_.begin();
	for(; superPixelIt != graph->superPixels_.end(); superPixelIt++) {
		superPixels.insert(superPixelIt->second);
	}

	// Create the segment
	Segment* newSegment = new Segment(superPixels);
	state->segments_.insert(newSegment);

	// Update the assignments of superpixels
	superPixelIt = graph->superPixels_.begin();
	for(; superPixelIt != graph->superPixels_.end(); superPixelIt++) 
		state->assignments_.insert(make_pair(superPixelIt->second, newSegment));

	return state;
}

long double Problem::jumpProbability (const Segment& sA1, const Segment& sB2, const Segment* sA2,
    const Segment* sB1) {

	// (1) Apply the division only to the guaranteed subgraphs to preserve precision
	long double sB2_sA1 = sB2.probabilityL_ - sA1.probabilityL_;

	// (2) Compute the second half with sA2 and sB2 if they exist
	long double sB1_sA2 = (sB1 ? sB1->probabilityL_ : 0.0) - (sA2 ? sA2->probabilityL_ : 0.0);

	// (3) Merge the two log ratios and convert them to base 10.
	long double ratioL = sB2_sA1 + sB1_sA2;
	return ratioL;
}
