/**
 * \file Serialization.h
 * \brief 
 *
 *  \date Apr 21, 2013
 *  \author Andrew Price
 */

#ifndef SERIALIZATION_H_
#define SERIALIZATION_H_

#include <boost/serialization/string.hpp>
#include <boost/graph/adj_list_serialize.hpp>

#include <Eigen/Core>
#include <iostream>

#include "SPGraph.h"

namespace boost
{
namespace serialization
{

template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options,
		int _MaxRows, int _MaxCols>
inline void serialize(Archive & ar,
		Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & t,
		const unsigned int file_version)
{
	size_t rows = t.rows(), cols = t.cols();
	ar & rows;
	ar & cols;
	if (rows * cols != t.size())
		t.resize(rows, cols);

	for (size_t i = 0; i < t.size(); i++)
		ar & t.data()[i];
}

template<class Archive>
inline void serialize(Archive& ar,
		pcl::PointXYZ& point,
		const unsigned int version)
{
	std::string label = "Point";
	//ar & label;
	ar & point.x;
	ar & point.y;
	ar & point.z;
}

template<class Archive>
inline void serialize(Archive& ar,
		pcl::PointXYZRGB& point,
		const unsigned int version)
{
	std::string label = "Point";
	//ar & label;
	ar & point.x;
	ar & point.y;
	ar & point.z;
	ar & point.r;
	ar & point.g;
	ar & point.b;
}

template<class Archive>
inline void serialize(Archive& ar,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
		const unsigned int version)
{
	//std::cerr << "cloud:";
	std::string label = "Cloud";
	ar & label;
	ar & cloud->height;
	ar & cloud->width;
	ar & cloud->is_dense;
	//std::cerr << cloud->height << "x" << cloud->width << "\t";
	ar & cloud->points;
}

template<class Archive>
inline void serialize(Archive& ar,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
		const unsigned int version)
{
	//std::cerr << "cloud:";
	std::string label = "Cloud";
	//ar & label;
	ar & cloud->height;
	ar & cloud->width;
	ar & cloud->is_dense;
	ar & cloud->points;
}

template<class Archive>
inline void serialize(Archive &ar, SPNodeState& nodeState, const unsigned int version)
{
	ar & nodeState.currentSegmentID;
	ar & nodeState.currentModel;
	ar & nodeState.currentModelParams;
}

template<class Archive>
inline void serialize(Archive &ar, SPEdgeState& edgeState, const unsigned int version)
{
	ar & edgeState.partitionOn;
}

template<class Archive>
inline void serialize(Archive &ar, SPNode& node, const unsigned int version)
{
	std::string label = "Node";
	//ar & label;
	ar & node.spid;
	ar & node.parentFrame;
	ar & node.modelParams;
	ar & node.position;
	ar & node.imagePosition;
	if (node.subCloud == NULL)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr subCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		node.subCloud = subCloud;
	}
	ar & node.subCloud;
	if (node.subCloudCartesian == NULL)
	{
		node.setSubCloud(node.subCloud);
	}
	std::cerr << node.spid << "\t";
}

template<class Archive>
inline void serialize(Archive &ar, SPEdge& edge, const unsigned int version)
{
	ar & edge.BernoulliProbability;
}

} // namespace serialization
} // namespace boost

#endif /* SERIALIZATION_H_ */
