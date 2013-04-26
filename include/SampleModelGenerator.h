/**
 * SampleModelGenerator.h
 *
 *  Created on: Apr 3, 2013
 *      Author: arprice
 */

#ifndef SAMPLEMODELGENERATOR_H_
#define SAMPLEMODELGENERATOR_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

class SampleModelGenerator
{
public:
	SampleModelGenerator();
	virtual ~SampleModelGenerator();

	pcl::PointCloud<pcl::PointXYZ>::Ptr GenerateSampleSphere(const Eigen::Isometry3f center, const float r, const float thetaRes, const float phiRes, const float sigma = 0.0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr GenerateSamplePlane(const Eigen::Isometry3f center, const float r, const float rRes, const float sigma = 0.0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr GenerateSampleCube(const Eigen::Isometry3f center, const float r, const float rRes, const float sigma = 0.0);

private:
	boost::mt19937 gen;
};

#endif /* SAMPLEMODELGENERATOR_H_ */
