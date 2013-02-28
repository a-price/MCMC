/**
 * @file IO.h
 * @date Nov 9, 2011
 * @author Can Erdogan
 * @author Manohar Paluri
 * @brief A class that implements the required IO functionalities.
 * @todo All these functions should return vectors or such, that are independent
 * of the classes that use them! We never know how we will need to read a pcd file!
 */

#pragma once

#include "Graph.h"
#include "Common.h"

#include "opencv2/core/core.hpp"

#include <gtsam/base/timing.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>

#include <png++/png.hpp>

#include <err.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include <iostream>
#include <iterator>
#include <fstream>
#include <sstream>
#include <string>

static const bool debugIOTiming = true;

/// The IO class
class IO {
public:

	// ==========================================================================
	// Read and write the raw data

	/// Writes the disparity and color values for each pixel in order, even those without depth info
	static void writeRawData (const cv::Mat& disparities, const cv::Mat& colors,
	    const std::string& filePath);

	/// Reads the disparity and color values as images
	static void readRawData (const std::string& filePath, cv::Mat& disparities, cv::Mat& colors);

public:

	// ==========================================================================
	// Writing PCD files

	/// Visualizes the point cloud with the given disparity and color values.
	static void writePCD (const std::string& filePath, const cv::Mat& disparities, const cv::Mat& colors,
	    uint16_t* conversionTable);

	/// Writes a pcd header
	static void writePCDHeader (FILE* file, size_t numPoints, bool withColor = false);

public:

	// ==========================================================================
	// Reading parameters

	/// Reads segmentation parameters
	static void readSegmentationParams (const std::string& filePath,
	    OverSegmentationParameters& params);

public:

	// ==========================================================================
	// Basic reading and writing functionalities

	/// Returns a file name with the current time
	static std::string createTimeFileName (std::string prefix = std::string());

	/// Given a line, reads all the doubles in the line and returns them in a vector.
	inline static void readDoublesFromLine (std::string line, std::vector <double>& doubles,
	    size_t numDoubles = 0) {

		// Create the stream
		std::stringstream stream(line, std::stringstream::in);

		// Read all the doubles
		double newDouble;
		while (stream >> newDouble) {
			doubles.push_back(newDouble);
		}

		// Return a subset of the doubles upon request
		if(numDoubles != 0) {
			doubles.erase(doubles.begin() + numDoubles, doubles.end());
		}
	}

	/**
	 * Get the number of lines in a file.
	 * @param filePath The path to the file
	 * @return The number of lines
	 */
	inline static size_t numLines (std::string filePath) {

		// Open the file
		std::fstream file(filePath.c_str(), std::fstream::in);
		if(!file.is_open()) throw Exception("Could not open the file!");

		// Read in each line
		size_t fileLines = 0;
		std::string dummy;
		while (getline(file, dummy))
			++fileLines;

		// Return the number of lines
		return fileLines;
	}

	/**
	 * Reads a matrix from a given file.
	 * @param filePath The path to the file
	 * @param data The matrix data which will be populated after the read.
	 * @param startLine The line from which the file should be processed from.
	 * @param numRowsLimit The number of rows (lines) that should be read at most.
	 * @param numColsLimit The number of cols (numbers) from a line that should be read at most.
	 */
	static void readMatrix (std::string filePath, gtsam::Matrix& data, size_t startLine = 0,
	    size_t numRowsLimit = 0, size_t numColsLimit = 0);
};
