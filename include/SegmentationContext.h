#ifndef SEGMENTATION_CONTEXT_H
#define SEGMENTATION_CONTEXT_H

#include "Common.h"
#include "Graph.h"
#include "IO.h"
#include "OverSegmentation.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>

#include <map>

class SegmentationContext
{
public:
	// Global superpixel lookup by u,v index
	Graph graph;
	cv::Mat overSegmented;
	Eigen::MatrixXf spLookup;
	std::map<SuperPixelID, int> SPtoS;
	std::vector<SuperPixelID> StoSP[4];
	// Currently selected S
	int currentS;
};

#endif // SEGMENTATION_CONTEXT_H
