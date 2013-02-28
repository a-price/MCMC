/**
 * @date May 30, 2012
 * @author Can Erdogan
 * @file Math.h
 * @brief Includes the common mathematical operations.
 */

#pragma once

#include "Common.h"

#include <gtsam/base/DerivedValue.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/GaussianDensity.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Values-inl.h>

#include "opencv2/core/core.hpp"
#define v3(p) (p).x(), (p).y(), (p).z()

static const double kDisparityVariance = 7;
static const double kBaseline = 0.10;							///< The baseline between the IR and RGB cameras
static const double kFocalLength = 580.0;					///< The focal length of IR camera in pixels
static const size_t kWidth = 640;									///< The number of pixels along the width
static const size_t kHeight = 480;								///< The number of pixels along the height


/// The definition which also prints the function name
#define Exception(x) (Exception_(__FUNCTION__, x))

/// The definition of a variable-string exception
struct Exception_ : public std::exception {
	std::string errorMessage;
	Exception_ (std::string func, std::string str) : errorMessage(func + ": " + str) {printf("func: %s\n", func.c_str());}
	~Exception_ () throw () {}
	const char* what () const throw () { return errorMessage.c_str(); }
};

/* ********************************************************************************************* */
/// Create a disparity image from 11-bit disparity values.
static cv::Mat* disparityImage (const cv::Mat& disparities) {

	// Find the minimum and maximum disparity
	short minDisparity = 2049, maxDisparity = 0;
	for(size_t j = 0, k = 0; j < 480; j++) {
		for(size_t i = 0; i < 640; i++, k++) {
			short disparity = disparities.at<short>(j,i);
			if(disparity == 0) continue;
			maxDisparity = std::max(disparity, maxDisparity);
			minDisparity = std::min(disparity, minDisparity);
		}
	}

	// Get the range of inverse disparities
	double minInvDisparity = 1.0 / maxDisparity;
	double maxInvDisparity = 1.0 / minDisparity;
	double range = maxInvDisparity - minInvDisparity;
	if(range <= 0.0) throw Exception("Negative range value");

	// Set the pixel values
	cv::Mat* normalized = new cv::Mat(480, 640, CV_8UC1); 
	for(size_t j = 0, k = 0; j < 480; j++) {
		for(size_t i = 0; i < 640; i++, k++) {

			// Get the normalized inverse disparity value
			short disparity = disparities.at<short>(j,i);
			double normInvDisparity = 1.0;
			if(disparity != 0) {
				double invDisparity = (1.0 / disparity);
				normInvDisparity = (invDisparity - minInvDisparity) / range;
				if(!((normInvDisparity >= 0.0) && (normInvDisparity <= 1.0)))
					throw Exception("Weird inverse disparity value");
			}

			// Set the image value
			size_t greyValue = 255 * std::min(1.0, normInvDisparity);
			normalized->at<char>(j,i) = greyValue;
		}
	}

	return normalized;
}

/* ********************************************************************************************* */
/// Transforms the disparity plane parameterization δ = a1*u + b1*v + c, to the explicit
/// parameterization d = ax + by + cz.
static gtsam::Vector disparityToExplicit (gtsam::Vector v) {

	double f = kFocalLength;
	double N = sqrt(v(0)*v(0) + v(1)*v(1) + ((v(2)/f) * (v(2)/f)));
	double a = v(0) / N;
	double b = v(1) / N;
	double c = v(2) / (f*N);
	double d = kBaseline / N;
	return gtsam::Vector_(4, a, b, c, d);
}

	/* ********************************************************************************************* */
/// The function that converts explicit parameterization to disparity parameterization
static gtsam::Vector explicitToDisparity (gtsam::Vector v) {

	double f = kFocalLength;
	double b = kBaseline;
	double a_l = (b*v(0))/v(3);
	double b_l = (b*v(1))/v(3);
	double c_l = (f*b*v(2))/v(3);
	return gtsam::Vector_(3, a_l, b_l, c_l);
}

/* ********************************************************************************************* */
/// Local fitting - given a linear problem, Ax=b with a noise model, returns 
/// the GaussianDensity at the solution
static gtsam::GaussianDensity solve (const gtsam::Matrix& A, const gtsam::Vector& b, double* error = NULL) {

	static const bool debug = 0;

	// Create the graph
	gtsam::GaussianFactorGraph graph;

	// Create the model
	gtsam::Vector variances = gtsam::repeat(A.rows(), kDisparityVariance);
	gtsam::SharedDiagonal model = gtsam::noiseModel::Diagonal::Variances (variances);

	if(debug) {
		std::cout << "\n\n\n\n";
		std::cout << "A = [" << A << "];" << std::endl;
		std::cout << "b = [" << b << "];" << std::endl;
	}

	// Add the Jacobian factor
	graph.add(0, A, b, model);

	// Solve the system and return the gaussian density
	gtsam::GaussianSequentialSolver solver (graph, true);
	gtsam::GaussianBayesNet::shared_ptr bayesNet = solver.eliminate();
	gtsam::GaussianDensity density = *(bayesNet->back());

	// Set the error if requested
	if(error != NULL) {
		gtsam::VectorValues values;
		values.insert(0, density.mean());
		*error = graph.error(values);
	}

	return density;

}

/// The plane information gathered after optimization
struct Plane {

	gtsam::GaussianDensity density_;							///< The density at the optimization point
	double error_;												///< The error of the surface fit

	/// The constructor
	Plane (const gtsam::Matrix& A, const gtsam::Vector& b) {
		density_ = solve(A, b, &error_);
	} 
 
	/// Copy construction
	Plane (const gtsam::GaussianDensity& density, double error) : density_(density), error_(error) {}
};


// struct SuperPixel;
typedef size_t SuperPixelID;							///< Typedef for a super pixel id

/// The superPixel structure that represents an atomic, homogeneous region.
struct SuperPixel {
	SuperPixelID id_;																	///< The ID of this super pixel
	Eigen::MatrixXd* A_;																				///< Pixel coordinates
	Eigen::VectorXd* b_;																				///< The disparities for the pixels
	std::map <SuperPixel*, long double> neighbors_;		///< The neighbors and edge prob. in the graph
	Plane* plane_; 																		///< The local plane
};



/* ********************************************************************************************* */
/// Global fitting - the total error and the local errors fields of the SurfaceInfo is updated.
static void fit (const std::map <SuperPixel*, gtsam::GaussianDensity>& densities,
 gtsam::GaussianDensity& density, double& totalError, std::map <SuperPixel*, double>* localErrors = NULL) {

	static const bool debug = 0;

	// Create the graph and the arbitrary index for global parameters
	gtsam::GaussianFactorGraph graph;
	gtsam::Index j = 0;

	// Get the dimensions of the surface that is being fit
	std::map <SuperPixel*, gtsam::GaussianDensity>::const_iterator densityIt = densities.begin();
	const gtsam::Vector& temp = densityIt->second.mean();
	size_t numDimensions = gtsam::dim(temp);

	// Set the initial linearization point
	gtsam::LieVector lin = gtsam::zero(numDimensions);

	// Add each surface to the graph as a prior Jacobian factor. Also, save which factor is used
	// for which superpixel to compute local error between global and local fit afterwards.
	std::map <SuperPixel*, gtsam::GaussianFactor*> factors;
	for (; densityIt != densities.end(); densityIt++) {

		// Add new Jacobian factor to the graph
		gtsam::JacobianFactor* factor = new gtsam::JacobianFactor(densityIt->second);
		factors.insert(std::make_pair(densityIt->first, factor));
		boost::shared_ptr <gtsam::JacobianFactor> factorPtr(factor);
		graph.add(factorPtr);

		if(debug) {

			// Local fitting error after optimization is assumed to be 0.
			gtsam::LieVector localStep = gtsam::zero(numDimensions);
			gtsam::Vector xi = localStep.localCoordinates(lin);

			// Compute the b vector
			const gtsam::Matrix& A = densityIt->second.get_R();
			gtsam::Vector b = densityIt->second.get_d() - A * xi;

			std::cout << "A: " << A << std::endl;
			std::cout << "\tAtA: \n" << A.transpose() * A << std::endl;
			std::cout << "\tRtR: \n" << densityIt->second.information() << std::endl;
			std::cout << "\td: " << densityIt->second.get_d().transpose() << std::endl;
			std::cout << "\txi: " << xi.transpose() << std::endl;
			std::cout << "\tb: " << b.transpose() << std::endl;

		}
	}

	// Optimize the graph and compute the result
	gtsam::GaussianSequentialSolver solver(graph, true);
	gtsam::GaussianBayesNet::shared_ptr bayesNet = solver.eliminate();
	density = *(bayesNet->back());

	// Get the local errors and total error
	gtsam::VectorValues values;
	values.insert(0, density.mean());
	std::map <SuperPixel*, gtsam::GaussianFactor*>::const_iterator factorIt = factors.begin();
	if(localErrors != NULL) *localErrors = std::map <SuperPixel*, double>();
	for (; factorIt != factors.end(); factorIt++) {
		double error = factorIt->second->error(values);
		if(debug) std::cout << "\t\tlocalError: " << error << std::endl;
		totalError += error;
		if(localErrors != NULL) localErrors->insert(std::make_pair(factorIt->first, error));
	}

	// TODO Remove this section - it is to confirm the output error is correct
	// by checking with every pixel
/*
	double realError = 0.0;
	gtsam::Vector _params = density.mean();
	Eigen::Vector3d params (_params(0), _params(1), _params(2));
	for (densityIt = densities.begin(); densityIt != densities.end(); densityIt++) {
		Eigen::VectorXd errors = (*densityIt->first->b_) - (*densityIt->first->A_) * params;  
		gtsam::Vector _lp = densityIt->first->plane_->density_.mean();
		std::cout << "\t\t\tlocal plane: " << _lp.transpose() << std::endl;
		Eigen::Vector3d localPlane (_lp(0), _lp(1), _lp(2));
		Eigen::VectorXd localerrors = (*densityIt->first->b_) - (*densityIt->first->A_) * localPlane;  
		double error = errors.squaredNorm();
		double localerror = localerrors.squaredNorm();
		realError += error;
		std::cout << "\t\tglobal local error real: " << error << " local plane error: " << localerror << std::endl;
	}
	
	std::cout << "realError: " << realError << " estimated: " << totalError << std::endl;
*/
	if(debug) printf("\t\t\ttotalError: %lf, graph.error: %lf\n", totalError, graph.error(values));
}



/* ********************************************************************************************* */
/// The addition function for log probabilities. If p1 and p2 are probabilities and we are
/// interested in log(p1 + p2) and know l1 = log(p1) and l2 = log(p2), we can rewrite log(p1+p2)
/// as log(p1) + log(1 + exp(log(p2) - log(p1))). Note that if l2 > l1, it is advised to switch
/// their order in practice because I guess doubles have more precision at the decimal places.
static long double addLogProbs (long double l1, long double l2) {

	// Decide which one is greater and assign it to lB and the other to lS.
	long double lB, lS;
	if(l1 > l2)
		lB = l1, lS = l2;
	else lS = l1, lB = l2;

	// Now the addition becomes (lB + log(1+exp(lS - lB))).
	long double sum = lB + log(1 + exp(lS - lB));
	if(isnan(sum)) throw Exception("Log prob. summation is nan!");
	return sum;
}

/* ********************************************************************************************* */
/// Given a vector, returns two orthogonal vectors to it.
static void orthogonalVectors (const gtsam::Point3& inputNormal, gtsam::Point3& vector1, gtsam::Point3& vector2) {

	// Normalize the input vector
	gtsam::Point3 normal = inputNormal / inputNormal.norm();

	// Create a second orthogonal vector by picking some vector that is not parallel to the normal
	// and computing their cross product
	gtsam::Point3 helperVector;
	if(normal.equals(gtsam::Point3(1.0, 0.0, 0.0)) || normal.equals(gtsam::Point3(-1.0, 0.0, 0.0)))
		helperVector = gtsam::Point3(0.0, 1.0, 0.0);
	else helperVector = gtsam::Point3(1.0, 0.0, 0.0);

	// Compute the first vector using the helper and the normal
	vector1 = helperVector.cross(normal);
	vector1 = vector1 / vector1.norm();

	// Compute the second vector using the normal and the first vector
	vector2 = normal.cross(vector1);
	vector2 = vector2 / vector2.norm();
}
