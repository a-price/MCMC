#include "../include/Graph.h"
#include "../include/IO.h"
#include "../include/PlaneLibrary.h"

#include <err.h>

using namespace std;
using namespace gtsam;

double Segment::mergeProbability (const Segment& segment,
    const set <SuperPixel*>& superPixels,
    const map <SuperPixel*, Segment*>& assignments) {

	double mergeProb = 1.0;
	set <SuperPixel*>::const_iterator superPixelIt = superPixels.begin();
	for(; superPixelIt != superPixels.end(); superPixelIt++) {

		// Traverse the neighbors of the superpixel
		map <SuperPixel*, long double>::const_iterator neighborIt =
				(*superPixelIt)->neighbors_.begin();
		for (; neighborIt != (*superPixelIt)->neighbors_.end(); neighborIt++) {

			// If the neighbor is not in the given subgraph, we skip. (It does not actually separate).
			Segment* neighborSegment = assignments.at(neighborIt->first);
			if(neighborSegment != &segment) continue;

			// Otherwise, if the neighbor is in the subgraph, we accumulate the (1 - edge probability).
			mergeProb *= (1.0 - neighborIt->second);
		}
	}

	return mergeProb;
}

Segment* Segment::combine (const Segment& segment, const set <SuperPixel*>& toAdd) {

	// Create the superpixels
	set <SuperPixel*> superPixels = segment.superPixels_;

	// Add the requested superpixels
	set <SuperPixel*>::const_iterator superPixelIt = toAdd.begin();
	for(; superPixelIt != toAdd.end(); superPixelIt++)
		superPixels.insert(*superPixelIt);

	// Create the new Segment
	Segment* merged = new Segment(superPixels);
	if(segment.color_ != NULL)
		merged->color_ = new cv::Vec3b(*segment.color_);
	return merged;
}

Segment* Segment::diff (const Segment& segment, const set <SuperPixel*>& toRemove) {

	// Create the superpixels
	set <SuperPixel*> superPixels = segment.superPixels_;

	// Remove the unwanted superpixels
	set <SuperPixel*>::const_iterator superPixelIt = toRemove.begin();
	for(; superPixelIt != toRemove.end(); superPixelIt++)
		superPixels.erase(*superPixelIt);

	// Create the new Segment and preserve its color
	Segment* split = new Segment(superPixels);
	if(segment.color_ != NULL)
		split->color_ = new cv::Vec3b(*segment.color_);
	return split;
}

Plane* Segment::fitGlobalSurface (Segment& segment, 
    PlaneLibrary* library) {

	static const bool debug = 0;

	// Case 1: Return the current surface info of the segment if available
	// NOTE I am assuming that we preserve the unaltered segments through the state transitions
	// and that they retrain their global surface information.
	if(segment.plane_ != NULL) {
		return segment.plane_;
	}

	// Case 2: Return the previously computed information from the library
	if(library != NULL) {
		Plane* plane = library->getInfo(segment);
		if(plane != NULL) {

			// Update the segment information and return it to the caller
			segment.plane_ = new Plane(plane->density_, plane->error_);
			return plane;
		}
	}

	// Case 3: Fit a global plane to the local planes of the segment

	// NOTE As we gather the densities, we accumulate the local errors
	// to incorporate into the global fitting error.
	double error = 0.0;
	map <SuperPixel*, GaussianDensity> densities;
	set <SuperPixel*>::const_iterator spIt = segment.superPixels_.begin();
	for(; spIt != segment.superPixels_.end(); spIt++) {
		densities.insert(make_pair(*spIt, (*spIt)->plane_->density_));
//		error += (*spIt)->plane_->error_;
	}

	fflush(stdout);
	// Fit the global plane and create a surface plane
	GaussianDensity density;
	fit(densities, density, error);
	segment.plane_ = new Plane(density, error);

	fflush(stdout);
	// Update the segment planermation, the library and return the created plane
	if(library != NULL) library->update(segment);
	return segment.plane_;
}

void Segment::computeProbability (Segment& segment, 
    PlaneLibrary* library) {

	static const bool debug = 0;

	// Fit the global surface
	Plane* plane = fitGlobalSurface(segment, library);
	if(debug) cout << "Global plane: " << plane->density_.mean().transpose() << endl;

	// Compute the log of the unnormalized probability
	const Matrix& cov = plane->density_.covariance();
	long double determinant = (2 * M_PI * cov).determinant();
	long double integration = sqrt(determinant);
	if(debug) printf("\tintegration: %Lf, integrationL: %Lf\n", integration, log(integration));
	if(debug) printf("\terror:       %lf\n", plane->error_);
	segment.probabilityL_ = -plane->error_ + log(integration);
	if(debug) printf("\t\tsegment prob: %Lf\n", segment.probabilityL_);

	if (isnan(segment.probabilityL_)) segment.probabilityL_ = 0.75;
	// Sanity check
	if(isnan(segment.probabilityL_)) throw Exception("The subgraph log probability is nan");
	if(isinf(segment.probabilityL_)) throw Exception("The subgraph log probability is inf");
}

void Segment::computeHash (Segment& segment) {

	// Define the static hash values
	static locale loc;
	static const collate <char>& coll = use_facet <collate <char> >(loc);

	// Traverse the superpixels in the order they are in the subgraph. They should not change
	// as the superpixel pointers are not modified after creation.
	set <SuperPixel*>::const_iterator superPixelIt = segment.superPixels_.begin();
	string buffer;
	char temp[256];
	for (; superPixelIt != segment.superPixels_.end(); superPixelIt++) {
		sprintf(temp, "%lu,", (*superPixelIt)->id_);
		buffer = buffer + string(temp);
	}

	// Make a hash
	segment.hash_ = coll.hash(buffer.data(), buffer.data() + buffer.length());
}

void Segment::print (const std::string& s, FILE* stream) const {
	fprintf(stream, "%s{", s.c_str());
	set <SuperPixel*>::iterator sp_it = superPixels_.begin();
	for (; sp_it != superPixels_.end(); sp_it++)
		fprintf(stream, "%lu, ", (*sp_it)->id_);
	fprintf(stream, "}");
}

void Segment::writeToPCD (const std::string& s) const {
}
