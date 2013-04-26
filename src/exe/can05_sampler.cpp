/*
 * @file main.cpp
 * @date Jan 21, 2012
 * @author Can Erdogan
 * @brief Creates a 3D segmentation with an oversegmented data read from an input file
 * using MCMC methods.
 */

#include "IO.h"
#include "MCMC.h"
#include "OverSegmentation.h"
#include "Problem.h"

#include <opencv2/highgui/highgui.hpp>
#include <time.h>

using namespace gtsam;
using namespace std;
using namespace Eigen;

/* ********************************************************************************************* */
/// Outputs the images of the best 'n' states 
void printResults (Statistics <Segmentation>& statistics, size_t numRequestedStates) {

	// Order the statistics
	vector <Statistics <Segmentation>::Entry> states;
	Statistics <Segmentation>::getOrderedStates(statistics, states);
	size_t numStates = states.size();
	if(states.size() < numRequestedStates)
		throw Exception("The number of requested states are more than the statistics size");

	// Traverse the states
	for(size_t i = 0; i < numRequestedStates; i++) {

		// Get the state and the corresponding image
		const Segmentation* state = states[numStates-i-1].object_;
		cv::Mat* image = Problem::visualize(*state); 

		// Write the image
		char buf [256];
		sprintf(buf, "./temp/best%lu.png", i);
		imwrite(buf, *image);
		delete image;
	}
}

/* ********************************************************************************************** */
/// The main script
int main(int argc, char* argv []) {

	// Sanity check for the input arguments
	srand(time(NULL));
	if(argc != 4) throw Exception("Usage: ./can05_sampler <# burn-in iters> <# main iters> <path_to_data_folder>");
	size_t numBurnInIters = atoi(argv[1]), numMainIters = atoi(argv[2]);
	string dataPath = string(argv[3]);

	// Output the input options for verification
	printf("Number of burn-in iterations: %lu\n", numBurnInIters);
	printf("Number of main iterations: %lu\n", numMainIters);
	printf("Data folder path: '%s'\n", dataPath.c_str());
	cout << "--------------------------------------------------" << endl;

	// Read the disparity and color images
	cv::Mat disparities, colors;
	IO::readRawData(dataPath + "/raw.txt", disparities, colors);

	// Read the segmentation parameters
	OverSegmentationParameters params;
	IO::readSegmentationParams("../data/overSegmentationParams.txt", params);
	cout << "Over-segmentation parameters: " << endl;
	params.print();
	cout << "--------------------------------------------------" << endl;

	/* ******************************** OVERSEGMENT AND SAMPLE ************************************ */

	// Perform segmentation and obtain graph
	Graph graph;
	cout << "Starting over-segmentation..." << endl;
	gttic_(OverSegmentation);
	OverSegmentation::overSegment(disparities, colors, params, graph);
	gttoc_(OverSegmentation);
	cout << "Over-segmentation completed with " << graph.superPixels_.size() << " superpixels." << endl;

	// Run the MCMC sampler over the segmentations of superpixels
	gttic_(Sampler_run);
	FastMetropolisHastings <Problem> sampler;
	Statistics <Segmentation> statistics = sampler.run(numBurnInIters, numMainIters, &graph);
	gttoc_(Sampler_run);

	/* *********************************** OUTPUT RESULTS ***************************************** */

	// Print the timing profile
	printf("Timing output: \n");
  tictoc_print_();
	cout << "--------------------------------------------------" << endl;

 	// Print the results
	if(system("rm -rf temp; mkdir temp;") == -1) throw Exception("The system command to create folder failed!");
	printResults(statistics, 1);

	return EXIT_SUCCESS;
}
