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

class GraphVisualization
{
public:
	GraphVisualization();
	virtual ~GraphVisualization();

	static std::vector<visualization_msgs::MarkerArray> VisualizeGraph(SPGraph graph);

	std::map<std::string, Eigen::Vector3i> colorMap;
};

#endif /* GRAPHVISUALIZATION_H_ */
