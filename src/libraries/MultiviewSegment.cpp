/**
 * \file MultiviewSegment.cpp
 * \brief 
 *
 *  \date Apr 24, 2013
 *  \author arprice
 */

#include "MultiviewSegment.h"

boost::shared_ptr<SPGraph> MultiviewSegment::mGraph;

MultiviewSegment::MultiviewSegment(long id)
{
	segmentID = id;
	hash = 0;
	probability = 1.0;
}

MultiviewSegment::MultiviewSegment(const MultiviewSegment& other)
{
	this->segmentID = other.segmentID;
	this->plane = other.plane;
	this->hash = other.hash;
	this->probability = other.probability;

	this->vertices = other.vertices;
}

void MultiviewSegment::computeFitPlane()
{
	/********** Concatenate Point Clouds **********/
	std::cerr << "Computing fit plane for segment " << segmentID << " with " << vertices.size() << " nodes...";
	if (vertices.size() == 0) return; // TODO: figure out what to do here...
	pcl::PointCloud<pcl::PointXYZ>::Ptr segmentCloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (std::set<SPGraph::vertex_descriptor>::iterator i = vertices.begin(); i != vertices.end(); i++)
	{
		SPNode& vertex = (*mGraph)[*i];
		if (vertex.subCloudCartesian == NULL)
		{
			if (vertex.subCloud == NULL)
			{
				std::cerr << "WTF?\n";
			}
			else
			{
				std::cerr << "Attempting repair.\n";
			}
			vertex.setSubCloud(vertex.subCloud);
		}
		(*segmentCloud) += *(vertex.subCloudCartesian);
	}

	std::cerr << "cat clouds. ";

	/********** Generate Plane Params **********/
	plane = MathUtils::fitPlaneSVD(segmentCloud);

	/********** Store Proposed Plane **********/
	for (std::set<SPGraph::vertex_descriptor>::iterator i = vertices.begin(); i != vertices.end(); i++)
	{
		SPNode& vertex = (*mGraph)[*i];
		vertex.proposedState->currentModelParams = plane.coefficients;
	}
	std::cerr << "success.\n";
}

long double MultiviewSegment::computeProbability()
{
	probability = 1.0;
	double localErr = 0;

	// Get total error for segment
	for (std::set<SPGraph::vertex_descriptor>::iterator i = vertices.begin(); i != vertices.end(); i++)
	{
		SPNode& vertex = (*mGraph)[*i];
		localErr += vertex.getErrorForPlaneModel(plane.coefficients);
	}

	// P(Zi|Si,Theta i)
	probability = exp(-localErr/2.0)*sqrt((2.0*M_PI)*plane.covariance.determinant());
	if (isnan(probability)) {std::cerr << " NaN detected. " << localErr << " " << plane.covariance.determinant();}
	if (probability < 1.0^-20) {std::cerr << " zero detected. " << probability << " " << localErr << " " << plane.covariance.determinant();}

	return probability;
}
