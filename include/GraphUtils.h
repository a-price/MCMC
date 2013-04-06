/*
 * GraphUtils.h
 *
 *  Created on: Apr 1, 2013
 *      Author: Andrew Price
 */

#ifndef GRAPHUTILS_H_
#define GRAPHUTILS_H_

//#include <ros/ros.h>

#include "Graph.h"
#include <map>
#include "SPGraph.h"

double pMerge(Eigen::VectorXf theta1, Eigen::VectorXf theta2,
		Eigen::VectorXf weights = Eigen::Vector4f::Ones(), double temperature=8);

double pMerge(SPNode a, SPNode b, double temperature=8);

Eigen::MatrixXf getSelfAdjacencyGraph(Eigen::MatrixXf& theta, Eigen::VectorXf weights, double minP = 0);

//Eigen::MatrixXf getPlanarAdjacencyGraph(Graph& spGraph, Eigen::MatrixXf& theta, Eigen::VectorXf weights, std::map<SuperPixelID, int>& spModelLookup);

SPGraph getPlanarAdjacencyGraph(Graph& graph,
		Eigen::MatrixXf& theta, Eigen::VectorXf weights,
		std::vector<Eigen::Vector2i> centroids2, std::vector<Eigen::Vector4f> centroids3,
		std::map<SuperPixelID, int>& spModelLookup);

void mergeNewScanGraph(SPGraph& original, SPGraph& incoming);

void getPairwiseAdjacencyGraph(const Eigen::MatrixXf& a, const Eigen::MatrixXf& b, Eigen::MatrixXf& adjacency);

void writeGraph(Eigen::MatrixXf& adjacency, std::string filename, std::string prefix = "a");

void writeOrderedGraph(Eigen::MatrixXf& adjacency, std::string filename, std::vector<Eigen::Vector2i> spCenters, std::string prefix = "a");

#endif /* GRAPHUTILS_H_ */
