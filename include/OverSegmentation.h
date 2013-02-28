/**
 * @file OverSegmentation.h
 * @author Can Erdogan
 * @author Manohar Paluri
 * @author Pedro Felzenszwalb
 * @date Apr , 2012
 * @brief The set of functions related with the oversegmentation of the RGB image using the
 * disparity information.
 * @note A section of the code has been translated from Felzenswalb's segmentation code.
 */

#pragma once

#include "../include/Common.h"
#include "../include/IO.h"
#include "../include/Segmentation/segment-graph.h"

#include <gtsam/base/Matrix.h>

using namespace gtsam;

class OverSegmentation {
public:

	/// Over-segments the color image using the disparities and outputs a graph of superpixels.
	static void overSegment(const cv::Mat& disparities, const cv::Mat& colors,
			const OverSegmentationParameters& params, Graph& graph);

	/// Creates a color image using the graph from overSegment
	static void visualize (const Graph& graph, cv::Mat& image);

public:

	// These function focus on creating the superpixels from the union-find universe

	/// Groups the pixels into their superpixel sets with find operations from the universe
	/// while also determining the neighbors of each superpixel. Additionally, saves the
	/// boundary pixels.
	static void createSuperPixels (universe& u, const cv::Mat& disparities,  
		std::map <SuperPixelID, SuperPixel*>& superPixels, std::set <size_t>& boundaryPixels); 

	/// Checks if a superpixel is linear - a horizontal or vertical line on the camera plane
	static bool isLinear (const gtsam::Matrix& matrix);

	/// Checks if all the pixels of a superpixel have zero disparity
	static bool isZeroDisparity (const gtsam::Vector& vector);
public:

	// These functions focus on the union-find operation

	/// Creates the edges from the disparity and color (possible smoothed) images
	static void createEdges (const cv::Mat& disparities, const cv::Mat& smoothed, 
			const OverSegmentationParameters& params, Edge*& edges);

	/// Computes the pixel distance using the spatial, disparity and distance options.
	static float computeWeight(const cv::Mat& disparities,
			const cv::Mat& colors, size_t x1, size_t y1, size_t x2, size_t y2,
			const OverSegmentationParameters& params); 

public:

	// These functions focus on the smoothing operation

	/// Performs cross bilateral filtering on the input color image using the disparity data.
	static void bilateralFilter(const cv::Mat& disparities, const cv::Mat& colors,
			cv::Mat& smoothed, size_t windowSize, double spatialStdev,
			double disparityStdev);

	/// Given the window size and the standard deviation, creates a 2D Gaussian filter.
	static Matrix createGaussianFilter(size_t windowSize, double stdev);

	/// Computes the 2D Gaussian value with the offset (x,y) from the mean and standard deviation s.
	/// NOTE If y is set to 0, it can be used as a 1D Gaussian.
	static inline double computeGaussian(double x, double y, double stdev) {
		double normConstant = 1.0 / (sqrt(2.0 * M_PI * stdev * stdev));
		double exponent = (x * x + y * y) / (2 * stdev * stdev);
		return normConstant * exp(-exponent);
	}
};
