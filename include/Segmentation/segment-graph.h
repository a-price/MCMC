/*
 Copyright (C) 2006 Pedro Felzenszwalb

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#ifndef SEGMENT_GRAPH
#define SEGMENT_GRAPH

#include <algorithm>
#include <cmath>
#include "disjoint-set.h"

// threshold function
#define THRESHOLD(size, c) (c/size)

/// The construct that connects two components (not necessarily pixels)
typedef struct {
	float weight;					///< Edge weight computed with the pixel properties (spatial, depth, etc.)
	int a, b;							///< Indices of the components that the edge connects
} Edge;

bool operator< (const Edge &a, const Edge &b) {
	return a.weight < b.weight;
}

/**
 * Returns a disjoint-set forest representing the segmentation.
 * @param num_vertices The number of vertices in graph.
 * @param num_edges The number of edges in the graph
 * @param edges The array of edges with the associated weights
 * @param c The constant for the threshold comparison.
 * @return The set of forests after union-find
 */
universe *segment_graph (int num_vertices, int num_edges, Edge *edges, float c, size_t minForestSize) {

	// Sort the edges by ascending weight
	std::sort(edges, edges + num_edges);

	// Make a disjoint-set forest; essentially, putting every vertex in its own forest.
	universe *u = new universe(num_vertices);

	// Set the initial threshold for all the vertices just by using the input constant. The thresholds
	// are used such that two components are merged if their common edge has a weight less than both
	// of their thresholds.
	float *thresholdColor = new float[num_vertices];
	for(int i = 0; i < num_vertices; i++) {
		thresholdColor[i] = THRESHOLD(1,c);
	}

	// Traverse the edges in the monotonically increasing weight order
	size_t counter = 0;
	for(int i = 0; i < num_edges; i++) {

		// Get the edge
		Edge *pedge = &edges[i];

		// Get the two components
		int comp1 = pedge->a, comp2 = pedge->b;

		// Get the roots of the components
		int comp1Root = u->find(comp1);
		int comp2Root = u->find(comp2);

		// If the two components are already in the same forest (merged), skip
		if(comp1Root == comp2Root) continue;

		// Otherwise, check their common edge and their thresholds to merge
		float threshold1 = thresholdColor[comp1Root];
		float threshold2 = thresholdColor[comp2Root];
		float edgeWeight = pedge->weight;

		// printf("%lu => components: (%d, %d), roots: (%d, %d), thresholds: (%f, %f), edge weight: %f\n", counter++, comp1,
		//		comp2, comp1Root, comp2Root, threshold1, threshold2, edgeWeight);
		// if(counter == 100) exit(0);
		if((edgeWeight <= threshold1) && (edgeWeight <= threshold2)) {

			// Merge the two components
			u->join(comp1Root, comp2Root);

			// Get the new root (from the merged components)
			int newRoot = u->find(comp1Root);

			// Update the threshold for the new forest using the size of the combined components
			size_t newSize = u->size(newRoot);
			thresholdColor[newRoot] = edgeWeight + THRESHOLD(newSize, c);
		}
	}

	// Free the thresholds
	delete thresholdColor;

	// Return the set of forests
	return u;
}

#endif
