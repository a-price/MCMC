/**
 * @file OverSegmentation.h
 * @author Can Erdogan
 * @date Apr 24, 2012
 */

#include "../include/OverSegmentation.h"
#include "../include/Problem.h"
#include "opencv2/highgui/highgui.hpp"

#include <gtsam/base/timing.h>

#include <iostream>

using namespace std;
using namespace gtsam;

/* ********************************************************************************************** */
void OverSegmentation::overSegment(const cv::Mat& disparities,
		const cv::Mat& colors, const OverSegmentationParameters& params,
		Graph& graph) {

	static const bool debug = 0;

	// 1. Smooth the color image using a bilateral filter
	gttic_(Bilateral_filter);
	cv::Mat smoothed;
	bilateralFilter(disparities, colors, smoothed, params.windowSize_,
			params.spatialStdev_, params.disparityStdev_);
	gttoc_(Bilateral_filter);
	std::cerr << "BL Filtered." << std::endl;
	if(debug) cv::imwrite("./smoothed.png", smoothed);

	// 2. Compute the weights of the edges between the pixels
	gttic_(Create_edges);
	size_t numPixels = smoothed.rows * smoothed.cols;
	Edge *edges = new Edge[numPixels * 4];
	size_t numEdges = createEdges (disparities, smoothed, params, edges);

	gttoc_(Create_edges);
	std::cerr << "Created Edges." << std::endl;
	// 3. Perform union-find with the edge weights
	gttic_(Union_find);
	universe *u = segment_graph(numPixels, numEdges, edges,
			params.weightThreshold_, 0);
	gttoc_(Union_find);
	std::cerr << "Graph Segmented." << std::endl;
	// 4. Collect the pixels into their superpixels and get the neighbors
	gttic_(Create_superpixels);
	createSuperPixels (*u, disparities, graph.superPixels_, graph.boundaryPixels_);
	gttoc_(Create_superpixels);
	std::cerr << "SP Created." << std::endl;
	// 5. Compute the edge probabilities
	Problem::computeEdgeProbs(graph);

	// TODO: Consider cleaning up memory 
}

/* ********************************************************************************************** */
void OverSegmentation::visualize(const Graph& graph, cv::Mat& image) {

	// Initialize the image with white
	image = cv::Mat::ones (kHeight, kWidth, CV_8UC3);
	for (int y = 0; y < kHeight; y++) 
		for (int x = 0; x < kWidth; x++) 
			image.at<cv::Vec3b>(y,x) = cv::Vec3b(255, 255, 255);

	// Set the superpixel colors
	map <SuperPixelID, SuperPixel*>::const_iterator superPixelIt = graph.superPixels_.begin();
	for(; superPixelIt != graph.superPixels_.end(); superPixelIt++) {

		// Create a random color
		size_t r = rand () % 256, g = rand () % 256, b = rand () % 256;

		// Set the colors for the pixels
		for(size_t pixelIdx = 0; pixelIdx < superPixelIt->second->A_->rows(); pixelIdx++) {
			size_t u = (*superPixelIt->second->A_)(pixelIdx, 0), v = (*superPixelIt->second->A_)(pixelIdx, 1);
			image.at<cv::Vec3b>(v,u) = cv::Vec3b(b,g,r);
		}
	}

	// Set the boundary pixels
	set <size_t>::const_iterator pixelIt = graph.boundaryPixels_.begin();
	for(; pixelIt != graph.boundaryPixels_.end(); pixelIt++) {
		size_t pixelIdx = *pixelIt;
		size_t u = pixelIdx % kWidth, v = pixelIdx / kWidth;
		image.at<cv::Vec3b>(v,u) = cv::Vec3b(0,0,0);
	}
}

/* ********************************************************************************************** */
void OverSegmentation::createSuperPixels (universe& u, const cv::Mat& disparities,  
		map <SuperPixelID, SuperPixel*>& superPixels, set <size_t>& boundaryPixels) {

	// Initialize the fields
	size_t previousIDs [kWidth + 1];
	int directions[] = { -1, 0, 0, -1, -1, -1, 1, -1 };		// left, top, top-left, top-right
	map <SuperPixel*, size_t> numRecordedPixels; 
	SuperPixel* sp;
	size_t index;
	std::cerr << "CSP Initialized." << std::endl;
	std::cerr << "Disp: " << disparities.rows << "x" << disparities.cols << std::endl;
	std::cerr << "SPs: " << superPixels.size() << std::endl;
	std::cerr << "Universe: " << u.num_sets() << std::endl;
	long int errCount = 0;
	// Traverse each pixel, creating superpixels if need be and
	// determining the neighbors
	for (size_t y = 0, k = 0; y < kHeight; y++) {
		//std::cerr << " " << k ;
		for (size_t x = 0; x < kWidth; x++, k++) {

			// Get the superpixel ID for this pixel
			SuperPixelID id = u.find(k);

			// =================================================
			// A. Place pixel in its superpixel

			// Get the superpixel for this pixel
			map <SuperPixelID, SuperPixel*>::iterator superPixelIt = superPixels.find(id);
			if(superPixelIt == superPixels.end()) {
				//std::cerr << "Creating SP: " << id << std::endl;
				// Create a new superpixel
				size_t numPixels = u.size(id);
				sp = new SuperPixel;
				superPixels[id] = sp;
				sp->id_ = id;
				sp->A_ = new Matrix();
				sp->b_ = new Vector();
				*(sp->A_) = zeros(numPixels, 3);
				*(sp->b_) = zero(numPixels);
				index = 0;
				numRecordedPixels[sp] = 1;
				//std::cerr << "Created SP: " << id << std::endl;
			}
	
			else {
				sp = superPixelIt->second;
				map <SuperPixel*, size_t>::iterator it = numRecordedPixels.find(sp);
				index = it->second;
				it->second++;
				if (index > 100 && index < 120)
				{
					//std::cerr << "SP*: " << sp << std::endl;
					//std::cerr << "idx: " << it->second << std::endl;
				}
			}

			// Get the (u,v) values and the disparity value for the pixel
			// TODO Use x,y instead of u,v which is already defined
			size_t v = k / kWidth, u = k % kWidth;
			size_t disparity = disparities.at<short>(v,u);
			if (index > 100 && index < 120)
			{
				//std::cerr << "Placing Pixel in SP: " << index << std::endl;
				//std::cerr << "A: " << (*sp->A_).rows() << "x" << (*sp->A_).cols() << std::endl;
				//std::cerr << "b: " << (*sp->b_).rows() << "x" << (*sp->b_).cols() << std::endl;
			}
			// Place the pixel in the superpixel
			if (index < sp->A_->rows())
			{
			(*sp->A_)(index,0) = u;
			(*sp->A_)(index,1) = v;
			(*sp->A_)(index,2) = 1.0;
			(*sp->b_)(index) = disparity;
			}
			else
			{
				//std::cerr << "Error: " << sp->id_ << "# " << index << std::endl;
				errCount++;
			}
			//if (index > 100 && index < 120)
				//std::cerr << "Placed Pixel in SP: " << index << std::endl;
			// =================================================
			// B. Determine the neighbors of its superpixel

			// Update the list of the last (w+1) pixels
			previousIDs[k % (kWidth + 1)] = id;

			// Compare the ID of this pixel with its 4 neighbors: LEFT, TOP, TOP-LEFT, TOP-RIGHT
			for(size_t i = 0; i < 4; i++) {

				// Check that the neighbor is within the image boundaries
				int x2 = x + directions[2 * i], y2 = y + directions[2 * i + 1];
				if ((x2 >= kWidth) || (y2 >= kHeight)) continue;

				// Determine the neighborhood status and set the boundary if necessary
				size_t k2 = x2 + y2 * kWidth;
				size_t id2 = previousIDs[k2 % (kWidth + 1)];
				if(id != id2) {
					SuperPixel* sp2 = superPixels.at(id2);
					sp->neighbors_.insert(make_pair(sp2, -1.0));
					sp2->neighbors_.insert(make_pair(sp, -1.0));
					boundaryPixels.insert(k);
				}
			} 
		}
	}
	std::cerr << std::endl << "CSP Split." << std::endl;
	// Compute the local planes
	map <SuperPixelID, SuperPixel*>::iterator spIt = superPixels.begin();
	while(spIt != superPixels.end()) {

		// Check the linearity and the disparity values
		sp = spIt->second;
		if((sp->A_->rows() < 10) || (isLinear(*(sp->A_))) || 
			(isZeroDisparity(*(sp->b_)))) { 

			// Remove the superpixel from the neighbor lists of its neighbors
			map <SuperPixel*, long double>::iterator it = sp->neighbors_.begin();
			for(; it != sp->neighbors_.end(); it++) {
				SuperPixel* neighbor = it->first;
				if(neighbor->neighbors_.erase(sp) != 1) 
					throw Exception("Could not remove bad superpixel from neighbor list");
			}		
	
			// Remove the superpixel from the list of superpixels
			superPixels.erase(spIt++);
		}
		
		// Compute the plane
		else {
			sp->plane_ = new Plane(*(sp->A_), *(sp->b_));
			++spIt;	
		}
	}

	// Check that all the superpixels have planes
	spIt = superPixels.begin();
	for(; spIt != superPixels.end(); spIt++) {
		if(spIt->second->plane_ == NULL) {
			printf("ha?!\n");
			exit(0);
		}
	}
	std::cerr << "Errors: " << errCount << std::endl;
}

/* ********************************************************************************************** */
float OverSegmentation::computeWeight(const cv::Mat& disparities,
		const cv::Mat& colors, size_t x1, size_t y1, size_t x2, size_t y2,
		const OverSegmentationParameters& params) {

	// Compute the RGB channel value differences between the two pixels
	cv::Vec3b c1 = colors.at<cv::Vec3b>(y1,x1);
	cv::Vec3b c2 = colors.at<cv::Vec3b>(y2,x2);
	float dr = ((float) c1[2]) - ((float) c2[2]);  
	float dg = ((float) c1[1]) - ((float) c2[1]);  
	float db = ((float) c1[0]) - ((float) c2[0]);  

	if(!((fabs(dr) <= 255.0) && (fabs(dg) <= 255.0) && (fabs(db) <= 255.0)))
		throw Exception("Wrong color difference computation!");

	// Get the disparity values (in float to avoid subtraction with unsigned)
	float disp1 = disparities.at<short>(y1,x1);
	float disp2 = disparities.at<short>(y2,x2);

	// Compute the color, depth and spatial values as the sum of squared differences
	float colorValue = sqrt(dr * dr + dg * dg + db * db);
	float disparityValue = fabs(disp1 - disp2);
	float distanceValue = sqrt(
			(x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

	// Combine the three metrics using the provided factors
	float edgeWeight = (params.colorFactor_ * colorValue
			+ params.disparityFactor_ * disparityValue
			+ params.distanceFactor_ * distanceValue);

	return edgeWeight;
}

/* ********************************************************************************************** */
size_t OverSegmentation::createEdges (const cv::Mat& disparities, const cv::Mat& smoothed, 
		const OverSegmentationParameters& params, Edge*& edges) {

	int directions[] = { 1, 0, 0, 1, 1, 1, 1, -1 };
	size_t numEntries = 0;
	for (int y = 0; y < kHeight; y++) {
		for (int x = 0; x < kWidth; x++) {

			// Make edges with the 4 neighboring pixels at the RIGHT, BOTTOM, BOTTOM-RIGHT and TOP-RIGHT
			for (size_t i = 0; i < 4; i++) {

				// Get the neighbor coordinates and make sure it is within the frame
				int x2 = x + directions[2 * i], y2 = y + directions[2 * i + 1];
				if ((x2 >= kWidth) || (y2 >= kHeight))	continue;

				// Set the vertex indices
				edges[numEntries].a = y * kWidth + x;
				edges[numEntries].b = y2 * kWidth + x2;
				assert((edges[numEntries].a >= 0) && (edges[numEntries].b >= 0) && "Can not create edges with < 0 neigh. indices");

				// Set the edge weight
				edges[numEntries].weight = computeWeight(disparities, smoothed, x, y, x2,
						y2, params);

				// Increment the counter
				numEntries++;
			}
		}
	}

	// Return the number of edges
	return numEntries;
}

/* ********************************************************************************************** */
void OverSegmentation::bilateralFilter(const cv::Mat& disparities,
		const cv::Mat& colors, cv::Mat& smoothed, size_t windowSize,
		double spatialStdev, double disparityStdev) {

	// Sanity checks
	if(spatialStdev <= 0.0) throw Exception("Non-positive standard deviation for the spatial Gaussian kernel");
	if(disparityStdev <= 0.0) throw Exception("Non-positive standard deviation for the disparity kernel");

	// If the window size is < 2, i.e. there is no kernel, skip.
	if (windowSize < 2) {
		smoothed = colors;
		return;
	}

	// Compute the spatial Gaussian beforehand as it does not change with the pixel location
	Matrix spatialKernel = createGaussianFilter(windowSize, spatialStdev);

	// Initialize the smoothed image
	smoothed = cv::Mat::zeros(kHeight, kWidth, CV_8UC3);

	// Move the spatial and depth smoothing filter through the pixels and apply bilateral to RGB
	// channels separately. Note that the right and bottom parts of the window are not be used.
	for (size_t y = 0; y < kHeight - windowSize; y++) {
		for (size_t x = 0; x < kWidth - windowSize; x++) {

			// 1. Get the "disparity-difference" kernel

			gttic_(Disparity_difference_kernel);

			// Get the disparity in the middle point - may need to take the mean if
			// window size is even
			size_t x0 = x + ceil(windowSize / 2.0), y0 = y + ceil(windowSize / 2.0);
			double mDisparity = disparities.at<short>(y0, x0);
			if (windowSize % 2 == 0)
				mDisparity = (disparities.at<short>(y0,x0) + 
											disparities.at<short>(y0,x0+1) + 
											disparities.at<short>(y0+1,x0) + 
											disparities.at<short>(y0+1,x0+1)) / 4.0; 

			// Compute the kernel values
			Matrix disparityKernel = zeros(windowSize, windowSize);
			for (int j = 0; j < windowSize; j++) {
				for (int i = 0; i < windowSize; i++) {

					// Evaluate the depth Gaussian (mu = 0, disparityStdev) for the depth difference between
					// the current pixel (i,j) and the center of the kernel (x,y)
					double disparityDiff = mDisparity - (double) disparities.at<short>(j+y, i+x); 
					double value = computeGaussian(disparityDiff, 0.0, disparityStdev);
					disparityKernel(i, j) = value;
				}
			}

			gttoc_(Disparity_difference_kernel);

			// 2. Multiply the disparity kernel with the spatial kernel and normalize it

			gttic_(Multiple_disparity_with_spatial_AND_normalize);
			Matrix jointKernel = spatialKernel.cwiseProduct(disparityKernel);
			double sum = jointKernel.sum();
			jointKernel /= sum;
			gttoc_(Multiple_disparity_with_spatial_AND_normalize);

			// 3. Convolve each RGB channel with the joint kernel within the window

			gttic_(Convolve_RGB_channels_with_joint_kernel);
			for (size_t channelIdx = 0; channelIdx < 3; channelIdx++) {

				// Create the matrix of values for the particular channel
				Matrix channel = zeros(windowSize, windowSize);
				for (int j = 0; j < windowSize; j++) {
					for (int i = 0; i < windowSize; i++) {
						channel(i,j) = colors.at<cv::Vec3b>(y+j,x+i)[channelIdx];
					}
				}

				// Convolve the joint kernel with the color image within the window
				double temp = channel.cwiseProduct(jointKernel).sum();
				size_t value = (size_t) temp;
				if(!((value >= 0) && (value < 256))) throw Exception("Weird channel value.");

				// Merge the current channel value with the already computed values
				smoothed.at<cv::Vec3b>(y,x)[channelIdx] = value;
			}
			gttoc_(Convolve_RGB_channels_with_joint_kernel);

		}
	}
}

/* ********************************************************************************************** */
Matrix OverSegmentation::createGaussianFilter(size_t windowSize, double stdev) {

	// Initialize the kernel
	Matrix kernel = zeros(windowSize, windowSize);

	// Compute the unnormalized kernel
	double sum = 0;
	double middlePoint = (1.0 + windowSize) / 2.0;
	for (size_t i = 1; i <= windowSize; i++) {
		for (size_t j = 1; j <= windowSize; j++) {

			// Compute the offsets from the middle point
			double x = middlePoint - i;
			double y = middlePoint - j;

			// Compute the Gaussian
			double value = computeGaussian(x, y, stdev);
			kernel(i - 1, j - 1) = value;
			sum += value;
		}
	}

	// Normalize the kernel and return it
	kernel = kernel / sum;
	return kernel;
}

/* ********************************************************************************************** */
bool OverSegmentation::isZeroDisparity (const Vector& d) {

	// All the entries in the disparity vector d, in A*\theta = d,
	// should be 0.
	for(size_t i = 0; i < d.rows(); i++) 
		if(d(i) >= 0.5) 
			return false;
	
	return true;

}

/* ********************************************************************************************** */
bool OverSegmentation::isLinear (const Matrix& matrix) {

	// If the superpixel is linear, then all the values along the first
	// or second column would be the same as the values in the first row
//	cout << "rows: " << matrix.rows() << endl;
  double u0 = matrix(0,0), v0 = matrix(0,1);
	bool foundDiffUs = false, foundDiffVs = false;
	for(size_t row_idx = 0; row_idx < matrix.rows(); row_idx++) {
		
		// Check if they are different
	//	cout << "diffs: " << fabs(matrix(row_idx,0)-u0) << " " << fabs(matrix(row_idx,1)-v0) << endl;
		if(fabs((double)matrix(row_idx,0)-u0) >= 0.5) foundDiffUs = true;
		if(fabs((double)matrix(row_idx,1)-v0) >= 0.5) foundDiffVs = true;

		// If both different u and v values are found, the superpixel is
		// guaranteed not to be horizontal or vertical
		if(foundDiffUs && foundDiffVs) return false;
	}

	return true;
}
