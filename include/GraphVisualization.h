/**
 * \file GraphVisualization.h
 * \brief 
 *
 *  \date Apr 5, 2013
 *  \author arprice
 */

#ifndef GRAPHVISUALIZATION_H_
#define GRAPHVISUALIZATION_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>

#include "SPGraph.h"
#include "MultiviewSegmentation.h"

class GraphVisualization
{
public:
	ros::NodeHandle nh;
	ros::Publisher nodePub;
	ros::Publisher edgePub;


	GraphVisualization();
	virtual ~GraphVisualization();

	static std::vector<visualization_msgs::MarkerArray> VisualizeGraph(SPGraph& graph);
	void VisualizeGraphStep(SPGraph& graph, std::set<SPGraph::vertex_descriptor>& proposedComponent, MultiviewSegmentation& segmentation);

	std::map<std::string, Eigen::Vector3i> colorMap;
};

#endif /* GRAPHVISUALIZATION_H_ */
