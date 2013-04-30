/**
 * \file SPGraph.cpp
 * \brief 
 *
 *  \date Apr 21, 2013
 *  \author Andrew Price
 */

#include "SPGraph.h"

SPNode::SPNode()
{
	currentState = boost::shared_ptr<SPNodeState>(new SPNodeState);
	proposedState = boost::shared_ptr<SPNodeState>(new SPNodeState);

	spid = 0;
}

void SPNode::acceptProposedState()
{
	// All we need to do here is flip the buffer
	// We could also clear out the proposed state, but probably unnecessary
	boost::shared_ptr<SPNodeState> temp = currentState;
	currentState = proposedState;
	proposedState = temp;
}

void SPNode::setSubCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud)
{
	// Create a deep copy for use in 3D-only operations
	// hopefully unnecessary in PCL 2.0
	subCloud = inCloud;
	subCloudCartesian = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*subCloud, *subCloudCartesian);
}


Eigen::VectorXf SPNode::getFullModel()
{
	Eigen::VectorXf retVal(modelParams.rows()+position.rows());
	retVal << modelParams, position;
	return retVal;
}

void SPNode::computeFullModel()
{
	/********** Generate Plane Params **********/
	Eigen::VectorXf fitPlane;
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
		model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (subCloudCartesian));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	ransac.setDistanceThreshold (.01);
	ransac.computeModel();
	ransac.getModelCoefficients(fitPlane);
	//ransac.getInliers()

	// Sanity checking
	if (fitPlane[0] != fitPlane[0]) {return;}
	if (fabs(fitPlane[0]) < 0.001) {return;}

	//TODO do an actual projection into z
	if (fitPlane[3] > 0.25) {fitPlane = -fitPlane;}

	/********** Generate Centroid **********/
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*subCloud, centroid);
	if (centroid.x() != centroid.x()) {return;}
	if (centroid(3) == 0) {centroid(3) = 1;}

	/********** Assign Model Params **********/
	modelParams = fitPlane;
	position = centroid.topRows(3);
}

Eigen::VectorXf SPNode::getDefaultWeights()
{
	Eigen::VectorXf retVal(modelParams.rows()+position.rows());
	retVal << 5.0, 5.0 ,5.0, 10.0,
			0.5, 0.5, 0.5;
	return retVal;
}

double SPNode::getErrorForProposedModel()
{
	return getErrorForPlaneModel(proposedState->currentModelParams);
}

double SPNode::getErrorForPlaneModel(Eigen::Vector4f& planeCoefficients)
{
	double sum = 0;
	double dist = 0;
	for (size_t i = 0; i < subCloud->points.size(); i++)
	{
		pcl::PointXYZRGB p = subCloud->points[i];
		//temp = pcl::pointToPlaneDistance(pixels->points[i],plane);
		dist = fabs(planeCoefficients[0] * p.x + planeCoefficients[1] * p.y + planeCoefficients[2] * p.z + planeCoefficients[3] );
		//dist *= dist;

		sum += dist;
	}
	//return sum;
	return sum/subCloud->points.size();
}

SPEdge::SPEdge()
{
	currentState = boost::shared_ptr<SPEdgeState>(new SPEdgeState);
	proposedState = boost::shared_ptr<SPEdgeState>(new SPEdgeState);

	BernoulliProbability = 0;
}

//SPEdge::~SPEdge()
//{
//	delete currentState;
//	delete proposedState;
//}

void SPEdge::acceptProposedState()
{
	// All we need to do here is flip the buffer
	// We could also clear out the proposed state, but probably unnecessary
	boost::shared_ptr<SPEdgeState> temp = currentState;
	currentState = proposedState;
	proposedState = temp;
}


bool SPEdgePredicate::operator() (const SPGraph::edge_descriptor edgeID) const
{
	SPEdge& edge = (*mGraph)[edgeID];
	return edge.currentState->partitionOn;
}
