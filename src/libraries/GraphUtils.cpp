/*
 * GraphUtils.cpp
 *
 *  Created on: Apr 1, 2013
 *      Author: Andrew Price
 */


#include "GraphUtils.h"

double pMerge(Eigen::VectorXf theta1, Eigen::VectorXf theta2,
		Eigen::VectorXf weights, double temperature)
{
	// TODO: Add Weight Matrix
	Eigen::VectorXf delta = (theta2-theta1).cwiseAbs();
	int size = delta.rows();
	Eigen::VectorXf error = weights.transpose() * Eigen::MatrixXf::Identity(size,size) * delta;
	double retVal = exp(-(error).squaredNorm() * temperature/2.0);
	return retVal;
}

Eigen::MatrixXf getSelfAdjacencyGraph(Eigen::MatrixXf& theta, Eigen::VectorXf weights, double minP)
{
	int n = theta.cols();
	int nnz = 0;
	Eigen::MatrixXf adjacency = Eigen::MatrixXf::Zero(n,n);
	for (long i = 0; i < n; i++)
	{
		Eigen::VectorXf a = theta.col(i);
		for (long j = i+1; j < n; j++)
		{
			Eigen::VectorXf b = theta.col(j);
			double p = pMerge(a, b, weights, 8.0);
			if (p > minP)
			{
				adjacency(i,j) = p;
				adjacency(j,i) = p;
				nnz++;
			}
		}
	}
	nnz *= 2;
	return adjacency;
}

Eigen::MatrixXf getPlanarAdjacencyGraph(Graph& spGraph, Eigen::MatrixXf& theta, Eigen::VectorXf weights, std::map<SuperPixelID, int>& spModelLookup)
{
	// TODO: This really should look up the model from the graph itself
	int n = theta.cols();
	int nnz = 0;
	Eigen::MatrixXf adjacency = Eigen::MatrixXf::Zero(n,n);

	std::map<SuperPixelID, SuperPixel*>::iterator sp;
	std::map<SuperPixelID, int>::iterator resultA;
	for (sp = spGraph.superPixels_.begin(); sp != spGraph.superPixels_.end(); ++sp)
	{
		resultA = spModelLookup.find(sp->first);
		if (resultA != spModelLookup.end())
		{
			std::map <SuperPixel*, long double> neighbors = sp->second->neighbors_;
			std::map <SuperPixel*, long double>::iterator neighbor;
			for (neighbor = neighbors.begin(); neighbor != neighbors.end(); ++neighbor)
			{
				std::map<SuperPixelID, int>::iterator resultB;
				resultB = spModelLookup.find(neighbor->first->id_);
				if (resultB != spModelLookup.end())
				{
					int i = resultA->second;
					int j = resultB->second;
					adjacency(i,j) = pMerge(theta.col(i), theta.col(j), weights, 8);
				}
			}

		}
	}

	nnz *= 2;
	return adjacency;
}

void getPairwiseAdjacencyGraph(const Eigen::MatrixXf& a, const Eigen::MatrixXf& b, Eigen::MatrixXf& adjacency)
{
	// TODO: incorporate xyz distance
	// TODO: incorporate nearest neighbor lookup
	int sa = a.cols();
	int sb = b.cols();
	int fullSize = sa + sb;
	Eigen::VectorXf weights = Eigen::VectorXf::Ones(a.rows());
	adjacency = Eigen::MatrixXf::Zero(fullSize, fullSize);

	for (int i = 0; i < sa; i++)
		for (int j = 0; j < sb; j++)
		{
			double p = pMerge(a.col(i), b.col(j), weights);
			if (p > 0.01)
			{
				adjacency(i,sa+j) = p;
				adjacency(sa+j,i) = p;
			}
		}
}

void writeGraph(Eigen::MatrixXf& adjacency, std::string filename, std::string prefix)
{
	int n = adjacency.cols();
	std::ofstream outfile;
	outfile.open(filename.c_str());
	outfile << "graph G {\n";

	for (int i = 0; i < n; i++)
	{
		outfile << prefix << i << ";\n";
	}
	for (long i = 0; i < n; i++)
	{
		for (long j = i+1; j < n; j++)
		{
			if (adjacency(i,j) != 0)
			{
				outfile << prefix << i << "--" << prefix << j << ";\n";
			}
		}
	}
	outfile << "}";


	outfile.close();
}

void writeOrderedGraph(Eigen::MatrixXf& adjacency, std::string filename, std::vector<Eigen::Vector2i> spCenters, std::string prefix)
{
	int n = adjacency.cols();
	int scale = 5;
	std::ofstream outfile;
	outfile.open(filename.c_str());
	outfile << "graph G {\n";

	for (int i = 0; i < n; i++)
	{
		outfile << prefix << i;
		outfile << "[label=\"\", pos = \"" << spCenters[i](0)*scale << "," << -spCenters[i](1)*scale << "!\"];\n";
	}
	for (long i = 0; i < n; i++)
	{
		for (long j = i+1; j < n; j++)
		{
			float weight = adjacency(i,j);
			if (weight != 0)
			{
				int width = (int)(weight*10.0)+1;
				if (isnan(width)) {width = 1;}
				outfile << prefix << i << "--" << prefix << j;
				outfile << "[penwidth=" << width << "]" << ";\n";

			}
		}
	}
	outfile << "}";


	outfile.close();
}
