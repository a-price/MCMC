/**
 * \file MathUtils.cpp
 * \brief 
 *
 *  \date Apr 26, 2013
 *  \author arprice
 */

#include "MathUtils.h"

PlaneModel::PlaneModel()
{
	covariance = Eigen::MatrixXf::Identity(3,3);
	error = 0;
}

double MathUtils::randbetween(double min, double max)
{
	return (max - min) * ( (double)rand() / (double)RAND_MAX ) + min;
}

PlaneModel MathUtils::fitPlaneSVD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	PlaneModel model;

	Eigen::MatrixXf cloudMat = cloud->getMatrixXfMap(3,4,0);

	Eigen::VectorXf centroid(cloudMat.rows());
	for (int i = 0; i < cloudMat.rows(); i++)
	{
		centroid(i) = cloudMat.row(i).mean();
		cloudMat.row(i) = cloudMat.row(i) - (Eigen::VectorXf::Ones(cloudMat.cols()).transpose()*centroid(i));
	}

	Eigen::JacobiSVD<Eigen::MatrixXf> svd(cloudMat, Eigen::ComputeFullU);
	Eigen::MatrixXf U = svd.matrixU();
	Eigen::VectorXf S = svd.singularValues();

	model.coefficients.topRows(3) = U.col(2);

	model.coefficients[3] = model.coefficients.topRows(3).dot(centroid);

	// Compute covariance matrix
	// C = U * S^2 * U^T / n-1
	model.covariance = U * (S.cwiseProduct(S)).asDiagonal() * U.transpose() / 2;

	// Compute total error
	for (int i = 0; i < cloudMat.cols(); i++)
	{
		model.error += pointToPlaneDistance(cloudMat.col(i), model.coefficients);
	}

	return model;
}

//	Eigen::VectorXf fitPlane;
//	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
//		model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (segmentCloud));
//
//	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
//	ransac.setDistanceThreshold (.01);
//	ransac.computeModel();
//	ransac.getModelCoefficients(fitPlane);
//
//	std::cerr << "got model. ";
//	// Sanity checking
//	if (fitPlane[0] != fitPlane[0]) {return;}
//	if (fabs(fitPlane[0]) < 0.001) {return;}
//
//	//TODO do an actual projection into z
//	if (fitPlane[3] < 0.0) {fitPlane = -fitPlane;}

float MathUtils::pointToPlaneDistance(Eigen::Vector3f p, Eigen::Vector4f planeCoefficients)
{
	return fabs(planeCoefficients[0] * p.x() + planeCoefficients[1] * p.y() + planeCoefficients[2] * p.z() + planeCoefficients[3] );
}

float MathUtils::pointToPlaneDistance(pcl::PointXYZ p, Eigen::Vector4f planeCoefficients)
{
	return fabs(planeCoefficients[0] * p.x + planeCoefficients[1] * p.y + planeCoefficients[2] * p.z + planeCoefficients[3] );
}
