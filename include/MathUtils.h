/**
 * \file MathUtils.h
 * \brief 
 *
 *  \date Apr 26, 2013
 *  \author arprice
 */

#ifndef MATHUTILS_H_
#define MATHUTILS_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <math.h>

class PlaneModel
{
public:
	Eigen::Vector4f coefficients;
	Eigen::MatrixXf covariance;
	double error;

	PlaneModel();
};

class MathUtils
{
public:
	static double randbetween(double min, double max);
	static PlaneModel fitPlaneSVD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	static inline float pointToPlaneDistance(Eigen::Vector3f p, Eigen::Vector4f planeCoefficients);
	static inline float pointToPlaneDistance(pcl::PointXYZ p, Eigen::Vector4f planeCoefficients);
};

#endif /* MATHUTILS_H_ */
