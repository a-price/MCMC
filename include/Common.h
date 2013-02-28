#pragma once

#include "MyMath.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>

#include <err.h>
#include <exception>
#include <signal.h>
#include <string>
#include <stdio.h>

struct SuperPixel;

/// The parameters used in oversegmentation
struct OverSegmentationParameters {

	size_t windowSize_;							///< The size of the kernel that filters the input images
	size_t weightThreshold_;				///< The threshold that controls the merging in the union-find
	double spatialStdev_;						///< The standard deviation of the spatial Gaussian
	double disparityStdev_;					///< The standard deviation of the disparity Gaussian
	double colorFactor_;						///< The importance of pixel color in neighborhoods
	double disparityFactor_;				///< The importance of disparity in pixel neighborhoods
	double distanceFactor_;					///< The importance of distance in pixel neighborhoods

	/// The default parameters
	OverSegmentationParameters () :
			windowSize_(0), weightThreshold_(0), spatialStdev_(0.0), disparityStdev_(
			    0.0), colorFactor_(0.0), disparityFactor_(0.0), distanceFactor_(0.0) {
	}

	/// Prints the parameters
	void print () const {
		std::cout << "windowSize: " << windowSize_ <<"\nspatialStdev: " << spatialStdev_ << "\ndisparityStdev: " <<
			disparityStdev_ << "\nweightThreshold: " << weightThreshold_ << "\ncolorFactor: " << colorFactor_ <<
			"\ndisparityFactor: " << disparityFactor_ << "\ndistanceFactor: " << distanceFactor_ << std::endl; 
	}	
};
