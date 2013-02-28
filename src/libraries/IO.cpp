#include "../include/IO.h"

using namespace gtsam;
using namespace std;

void IO::readRawData (const std::string& filePath, cv::Mat& disparities, cv::Mat& colors) {

	// Read the data file
	Matrix data;
	readMatrix(filePath, data);

	// Set the image properties
	disparities = cv::Mat(kHeight, kWidth, CV_16UC1);
	colors = cv::Mat(kHeight, kWidth, CV_8UC3);

	// Set the pixel values
	for(size_t v = 0, i = 0; v < kHeight; v++) {
		for(size_t u = 0; u < kWidth; u++, i++) {
			disparities.at<short>(v,u) = (short) data(i,0);
			size_t color = (size_t) data(i,1);
			cv::Vec3b colorv;
			colorv[0] = color & 0xff;
			colorv[1] = (color >> 8) & 0xff;
			colorv[2] = (color >> 16) & 0xff;
			colors.at<cv::Vec3b>(v,u) = colorv;
		}
	}
}

void IO::writeRawData (const cv::Mat& disparities, const cv::Mat& colors, const std::string& filePath) {

	// Open the file
	FILE* file = fopen(filePath.c_str(), "w");
	if(file == NULL) throw Exception("Could not open the raw data file.");

	// Write the data
	for(size_t v = 0; v < disparities.rows; v++) {
		for(size_t u = 0; u < disparities.cols; u++) {
			size_t disp = disparities.at<short>(v,u);
			cv::Vec3b colorv = colors.at<cv::Vec3b>(v,u);
			size_t color = (colorv[2] << 16) | (colorv[1] << 8) | colorv[0];
			fprintf(file, "%lu\t%lu\n", disp, color); 
		}
	}

	// Close the file
	if(fclose(file) != 0) throw Exception("Could not close the raw data file.");
}

void IO::writePCD (const string& filePath, const cv::Mat& disparities, const cv::Mat& colors,
    uint16_t* conversionTable) {

	// Open the file
	FILE* file = fopen(filePath.c_str(), "w");
	if(file == NULL) throw Exception("Could not open the pcd file.");

	// Write the pcd header with
	size_t numPixels = colors.rows * colors.cols;
	size_t numDigits = (size_t) ceil(log10(numPixels));
	writePCDHeader(file, numPixels, true);

	// Write the points
	size_t skipped = 0;
	int center_v = colors.rows / 2, center_u = colors.cols / 2;
	for(int v = 0; v < colors.rows; v++) {
		for(int u = 0; u < colors.cols; u++) {

			// Get the disparity and skip if no disparity information
			size_t disparity = disparities.at<short>(v,u);
			if((disparity == 2047) || (disparity == 0)) {
				skipped++;
				continue;
			}

			// Compute the XYZ values
			double z = static_cast <double>(conversionTable[disparity]);
			double x = (double) ((center_u - u) - 0.5) * z / kFocalLength;
			double y = (double) ((center_v - v) - 0.5) * z / kFocalLength;

			// Get the color value in unsigned int format
			cv::Vec3b color = colors.at<cv::Vec3b>(v,u);
			size_t red = color[2];
			size_t green = color[1];
			size_t blue = color[0];
			unsigned int colorValue = red * 256 * 256 + green * 256 + blue;

			// Write the coordinates and color value
			fprintf(file, "%.3f\t%.3f\t%.3f\t%u\n", x, y, z, colorValue);
		}
	}

	// Rewrite the number of points (assumes at most one digit decrease)
	fseek(file, 84, SEEK_SET);
	size_t newNumPixels = numPixels - skipped;
	size_t newNumDigits = (size_t) ceil(log10(newNumPixels));
	size_t diff = numDigits - newNumDigits;
	fprintf(file, "%lu%s", newNumPixels, (diff > 0) ? " " : "");
	fseek(file, 122, SEEK_SET);
	fprintf(file, "%lu%s", newNumPixels, (diff > 0) ? " " : "");

	// Close the file
	if(fclose(file) != 0) throw Exception("Could not close the pcd file.");
}

void IO::readSegmentationParams (const string& filePath, OverSegmentationParameters& params) {

  // Open the file
  ifstream ifs (filePath.c_str(), ifstream::in);
  if(ifs.fail()) throw Exception("Could not open the parameters file");

  // Read the lines
  char _line[1024];
  string lines [8];
  for(size_t i = 0; i < 7; i++) {
    ifs.getline(_line, 1024);
    if(ifs.fail()) throw Exception("Missing line"); 
    lines[i] = string(_line);
  }

  // Write the parameters
  params.windowSize_ = atoi(lines[0].substr(0, lines[0].find('#') - 1).c_str());
  params.spatialStdev_ = atof(lines[1].substr(0, lines[1].find('#') - 1).c_str());
  params.disparityStdev_ = atof(lines[2].substr(0, lines[2].find('#') - 1).c_str());
  params.weightThreshold_ = atoi(lines[3].substr(0, lines[3].find('#') - 1).c_str());
  params.colorFactor_ = atof(lines[4].substr(0, lines[4].find('#') - 1).c_str());
  params.disparityFactor_ = atof(lines[5].substr(0, lines[5].find('#') - 1).c_str());
  params.distanceFactor_ = atof(lines[6].substr(0, lines[6].find('#') - 1).c_str());

//	printf("%lu %lf %lf %lu %lf %lf %lf %lu\n", params.windowSize_, params.spatialStdev_, params.disparityStdev_,
//			params.weightThreshold_, params.colorFactor_, params.disparityFactor_, params.distanceFactor_);

	// Close the file
	ifs.close();
}

/// Returns a file name with the current time
string IO::createTimeFileName (string prefix) {

	// Get the current time
	time_t rawtime;
	time(&rawtime);
	struct tm* timeinfo = localtime(&rawtime);

	// Create the file name
	char fileName[128];
	strftime(fileName, 128, "%y%m%d-%H%M%S", timeinfo);
	return prefix + fileName;
}

void IO::writePCDHeader (FILE* file, size_t numPoints, bool color) {

	fprintf(file, "VERSION 0.7\n");
	fprintf(file, color ? "FIELDS x y z rgb\n" : "FIELDS x y z\n");
	fprintf(file, color ? "SIZE 4 4 4 4\n" : "SIZE 4 4 4\n");
	fprintf(file, color ? "TYPE F F F U\n" : "TYPE F F F\n");
	fprintf(file, color ? "COUNT 1 1 1 1\n" : "COUNT 1 1 1\n");
	fprintf(file, "WIDTH 1\n");
	fprintf(file, "HEIGHT %lu\n", numPoints);
	fprintf(file, "VIEWPOINT 0 0 0 1 0 0 0\n");
	fprintf(file, "POINTS %lu\n", numPoints);
	fprintf(file, "DATA ascii\n");
}

void IO::readMatrix (string filePath, gtsam::Matrix& data, size_t startLine, size_t numRowsLimit,
    size_t numColsLimit) {

	// Initialize variables
	char line[1024];
	vector <double> doubles;
	size_t numRows = 0, numCols = 0;

	// Get the number of lines unless specified with numRowsLimit parameter
	if(numRowsLimit == 0)
		numRows = numLines(filePath) - startLine;
	else numRows = numRowsLimit;

	// Open the file
	ifstream ifs(filePath.c_str(), ifstream::in);
	if(ifs.fail()) throw Exception("Could not open file the given file");

	// Move to the start line if one is specified
	if(startLine != 0) {
		for(size_t i = 0; i < startLine; i++)
			ifs.getline(line, 1024);
	}

	// Get the number of columns by reading the first line unless specified with numColsLimit parameter
	if(numColsLimit == 0) {
		ifs.getline(line, 1024);
		readDoublesFromLine(line, doubles);
		numCols = doubles.size();
	}
	else numCols = numColsLimit;

	// Initialize the matrix
	data = gtsam::zeros(numRows, numCols);

	// Set the first line values if necessary
	if(!doubles.empty()) {
		for(size_t i = 0; i < numCols; i++)
			data(0, i) = doubles[i];
	}

	// Read the file line by line and populate the data matrix
	size_t lineIndex = doubles.empty() ? 0 : 1;
	while (ifs.good()) {

		// Check if should stop
		if((numRowsLimit != 0) && (lineIndex >= numRowsLimit)) break;

		// Attempt to read the line
		ifs.getline(line, 1024);
		if(!ifs.good()) break;

		// Get the doubles from the line
		doubles.clear();
		readDoublesFromLine(line, doubles, numColsLimit);

		// Save the doubles to the data
		for(size_t i = 0; i < numCols; i++) {
			data(lineIndex, i) = doubles[i];
		}
		lineIndex++;
	}

	ifs.close();
}

