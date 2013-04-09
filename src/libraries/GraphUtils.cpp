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
	Eigen::VectorXf error = weights.cwiseProduct(delta);
	double retVal = exp(-(error).squaredNorm() * temperature/2.0);
	return retVal;
}

double pMerge(SPNode a, SPNode b, double temperature)
{
	// TODO: Add Weight Matrix
	return pMerge(a.getFullModel(), b.getFullModel(), a.getDefaultWeights(), temperature);
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
/*
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
*/

SPGraph getPlanarAdjacencyGraph(Graph& graph,
		Eigen::MatrixXf& theta, Eigen::VectorXf weights,
		std::vector<Eigen::Vector2i> centroids2, std::vector<Eigen::Vector4f> centroids3,
		std::map<SuperPixelID, int>& spModelLookup, double mergeThreshold)
{
	// TODO: This really should look up the model from the graph itself
	int n = theta.cols();
	SPGraph spGraph;
//	Eigen::MatrixXf adjacency = Eigen::MatrixXf::Zero(n,n);

	std::map<SuperPixelID, SuperPixel*>::iterator sp;
	std::map<SuperPixelID, int>::iterator ranPlaneModelIdx;
	std::map<SuperPixelID, SPGraph::vertex_descriptor> graphLookup;

	for (sp = graph.superPixels_.begin(); sp != graph.superPixels_.end(); ++sp)
	{
		ranPlaneModelIdx = spModelLookup.find(sp->first);
		if (ranPlaneModelIdx != spModelLookup.end())
		{
			SPGraph::vertex_descriptor v = boost::add_vertex(spGraph);
			graphLookup.insert(std::pair<SuperPixelID, SPGraph::vertex_descriptor>(sp->first, v));
			spGraph[v].spid = sp->first;
			spGraph[v].modelParams = theta.col(ranPlaneModelIdx->second);
			spGraph[v].position = centroids3[ranPlaneModelIdx->second].topRows(3);
			spGraph[v].imagePosition = centroids2[ranPlaneModelIdx->second];

			// TODO: Add other parameters
		}
	}


	for (sp = graph.superPixels_.begin(); sp != graph.superPixels_.end(); ++sp)
	{
		ranPlaneModelIdx = spModelLookup.find(sp->first);
		if (ranPlaneModelIdx != spModelLookup.end())
		{
			std::map <SuperPixel*, long double> neighbors = sp->second->neighbors_;
			std::map <SuperPixel*, long double>::iterator neighbor;
			for (neighbor = neighbors.begin(); neighbor != neighbors.end(); ++neighbor)
			{
				std::map<SuperPixelID, int>::iterator resultB;
				resultB = spModelLookup.find(neighbor->first->id_);
				if (resultB != spModelLookup.end())
				{
					//int i = resultA->second;
					//int j = resultB->second;
					//adjacency(i,j) = pMerge(theta.col(i), theta.col(j), weights, 8);

					// Lookup the vertex descriptors for the given SPIDs
					SPGraph::vertex_descriptor i = graphLookup.find(sp->first)->second;
					SPGraph::vertex_descriptor j = graphLookup.find(neighbor->first->id_)->second;

					// Connect the vertices with probability pMerge
					SPGraph::edge_descriptor e = boost::add_edge(i, j, spGraph).first; // get a new edge
					double p = pMerge(theta.col(i), theta.col(j), weights, 8);

					if (p > mergeThreshold)
					{
						spGraph[e].BernoulliProbability = p;
					}
				}
			}

		}
	}

	return spGraph;
}

void mergeNewScanGraph(SPGraph& original, SPGraph& incoming, double mergeThreshold)
{
	std::map<SPGraph::vertex_descriptor, SPGraph::vertex_descriptor> newNodeIDs;

	// add new nodes to the graph and compute edges as we go
	SPGraph::vertex_iterator vertexItA, vertexEndA;
	boost::tie(vertexItA, vertexEndA) = boost::vertices(incoming);
	for (; vertexItA != vertexEndA; ++vertexItA)
	{
		SPGraph::vertex_descriptor vertexIDI = *vertexItA; // dereference vertexIt, get the ID
		SPNode& vertexA = incoming[vertexIDI];

		// add incoming to original
		SPGraph::vertex_descriptor vertexIDA = boost::add_vertex(original);
		original[vertexIDA] = vertexA;

		// add to lookup
		newNodeIDs.insert(std::pair<SPGraph::vertex_descriptor, SPGraph::vertex_descriptor>(vertexIDI, vertexIDA));

		// switch order so we can add incoming as we go...

		SPGraph::vertex_iterator vertexItB, vertexEndB;
		boost::tie(vertexItB, vertexEndB) = boost::vertices(original);
		for (; vertexItB != vertexEndB; ++vertexItB)
		{
			SPGraph::vertex_descriptor vertexIDB = *vertexItB; // dereference vertexIt, get the ID
			SPNode& vertexB = original[vertexIDB];
			double p = pMerge(vertexA, vertexB);

			if (p > mergeThreshold)
			{
				SPGraph::edge_descriptor e = boost::add_edge(vertexIDA, vertexIDB, original).first; // get a new edge
				original[e].BernoulliProbability = p;

			}
		}
	}

	// relink original edges from incoming graph
	boost::tie(vertexItA, vertexEndA) = boost::vertices(incoming);
	for (; vertexItA != vertexEndA; ++vertexItA)
	{
		SPGraph::vertex_descriptor vertexIDI = *vertexItA;
		SPGraph::vertex_descriptor vertexIDA = newNodeIDs.find(vertexIDI)->second;

		// link through all neighbor vertices
		SPGraph::out_edge_iterator outEdgeIt, outEdgeEnd;
		boost::tie(outEdgeIt, outEdgeEnd) = boost::out_edges(vertexIDI, incoming);
		for (; outEdgeIt != outEdgeEnd; ++outEdgeIt)
		{
			SPGraph::vertex_descriptor neighborID = boost::target(*outEdgeIt, incoming);
			SPGraph::vertex_descriptor vertexIDB = newNodeIDs.find(neighborID)->second;
			SPGraph::edge_descriptor oldE = boost::edge(vertexIDI, neighborID, incoming).first;

			// todo check for existing

			SPGraph::edge_descriptor newE = boost::add_edge(vertexIDA, vertexIDB, original).first; // get a new edge
			original[newE].BernoulliProbability = incoming[oldE].BernoulliProbability;
		}
	}
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

void convertAtoG(Eigen::MatrixXf& adjacency, SPGraph& graph)
{
	int n = adjacency.cols();
	for (long i = 0; i < n; i++)
	{
		SPNode node;
		SPGraph::vertex_descriptor vID = boost::add_vertex(node, graph);
		graph[vID].numPixels = vID;
	}

	for (long i = 0; i < n; i++)
	{
		for (long j = i+1; j < n; j++)
		{

		}
	}
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

SPGraph generateSampleGraph()
{
	SPGraph graph;

	for(int i = 0; i < 8; i++)
	{
		SPGraph::vertex_descriptor v = boost::add_vertex(graph);

		// Creates a cube of side length size
		float size = 0.5;
		Eigen::Vector3f pos;
		pos << (size * ((i & 0x4) ? -1 : 1)),
			(size * ((i & 0x2) ? -1 : 1)),
			(size * ((i & 0x1) ? -1 : 1));

		Eigen::Vector4f theta;
		theta << pos[0],-pos[1],pos[2],1;

		std::cerr << pos.transpose() << std::endl;
		graph[v].position = pos;
		//std::cerr << graph[v].position.transpose() << std::endl;
		graph[v].modelParams = theta;

	}


	SPEdge edge;
	// Draw strong lines between top nodes
	for (int i = 0; i < 4; i++)
	{
		std::pair<SPGraph::edge_descriptor, bool> e = boost::add_edge(i, (i+1)%4, graph);
		SPGraph::edge_descriptor eID = e.first;
		graph[eID].BernoulliProbability = 0.9;
	}

	// Draw medium lines between levels
	for (int i = 0; i < 4; i++)
	{
		std::pair<SPGraph::edge_descriptor, bool> e = boost::add_edge(i, i+4, graph);
		SPGraph::edge_descriptor eID = e.first;
		graph[eID].BernoulliProbability = 0.5;
	}

	// Draw weak lines around bottom nodes
	for (int i = 0; i < 4; i++)
	{
		std::pair<SPGraph::edge_descriptor, bool> e = boost::add_edge(i+4, ((i+1)%4)+4, graph);
		SPGraph::edge_descriptor eID = e.first;
		graph[eID].BernoulliProbability = 0.2;
	}

	return graph;
}
