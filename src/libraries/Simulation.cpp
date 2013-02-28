#include "Simulation.h"

using namespace std;
using namespace Eigen;

namespace simulation {

	/* ******************************************************************************************* */
	// Oversegmentation

	void getSurfaces (vector <Surface*>& surfaces) {

		// Sanity check
		if(!surfaces.empty()) throw Exception("The input surfaces vector is not empty.");

		// Get the pixel data
		unsigned char* im = new unsigned char [3*cameraProperties.width_*cameraProperties.height_];
		glReadPixels(0,0,cameraProperties.width_, cameraProperties.height_, GL_RGB, GL_UNSIGNED_BYTE, im);

		// Create the surfaces by traversing each pixel and looking at their color values. Note that
		// multiple surfaces can share the same color.
		bool debug = false;
		size_t i = 0, w = cameraProperties.width_;
		map <size_t, string> usedLabels, availableLabels;
		size_t* labels [cameraProperties.height_][cameraProperties.width_];
		for(size_t y = 0; y < cameraProperties.height_; y++) {
			for(size_t x = 0; x < cameraProperties.width_; x++, i += 3) {

				// Check if this pixel belongs to the same surface as the left or the top pixelset
				size_t* topLabel = NULL, *leftLabel = NULL;
				if((x > 0) && (im[i] == im[i-3]) && (im[i+1] == im[i-2]) && (im[i+2] == im[i-1]))
					topLabel = labels[y][x-1];
				size_t j = i - 3*w;
				if((y > 0) && (im[i] == im[j]) && (im[i+1] == im[j+1]) &&  (im[i+2] == im[j+2]))
					leftLabel = labels[y-1][x];

				// Combine the top and left surfaces
				if((topLabel != NULL) && (leftLabel != NULL) && (*topLabel != *leftLabel)) {
					if(usedLabels.erase(*topLabel) != 1) throw Exception("Could not erase the topLabel");
					size_t temp = *topLabel;
					availableLabels.insert(make_pair(temp, ""));
					*topLabel = *leftLabel;
					labels[y][x] = leftLabel;
				}

				// Combine with the top surface
				else if (topLabel != NULL) labels[y][x] = topLabel;

				// Combine with the left surface
				else if (leftLabel != NULL) labels[y][x] = leftLabel;

				// Create a new surface from the available labels
				else if (!availableLabels.empty()) {
					size_t* label = new size_t;
					*label = availableLabels.begin()->first;
					if(availableLabels.erase(*label) != 1) throw Exception("Could not erase the label");
					usedLabels.insert(make_pair(*label, ""));
					labels[y][x] = label;
				}

				// Create a new surface with a new label
				else {
					size_t* label = new size_t;
					*label = usedLabels.size() + 1;
					usedLabels.insert(make_pair(*label, ""));
					labels[y][x] = label;
				}
			}
		}

		// Create a mapping from labels to indices to the surfaces vector and initialize surfaces
		map <size_t, size_t> labelsMapping;
		i = 0;
		for(map <size_t, string>::iterator it = usedLabels.begin(); it != usedLabels.end(); it++) {
			surfaces.push_back(new Surface);
			labelsMapping.insert(make_pair(it->first, i++));
		}

		// Create the surface structures
		for(size_t y = 0, i = 0; y < cameraProperties.height_; y++) {
			for(size_t x = 0; x < cameraProperties.width_; x++, i += 3) {
				if((im[i] == 0) && (im[i+1] == 0) && (im[i+2] == 0)) continue;
				size_t label = *(labels[y][x]);
				surfaces[labelsMapping[label]]->insert(make_pair(Vector2i(x,y), UNDEFINED_LABEL));
			}
		}

		// Remove the black surface which would be empty and set the surface colors
		for(vector <Surface*>::iterator it = surfaces.begin(); it < surfaces.end(); it++) {

			// Remove the black surface
			if((*it)->empty()) {
				it = surfaces.erase(it);
			}

			// Set the color
			Vector2i pixel = (*it)->begin()->first;
			size_t k = (pixel(1) * cameraProperties.width_ + pixel(0)) * 3;
			(*it)->color_ = Vector3d(im[k], im[k+1], im[k+2]);
		}

	}

	void oversegment (vector <SuperPixel*>& allSuperPixels) {

		printf("Starting oversegmentation... "); fflush(stdout);

		srand(time(NULL));

		// Create the surfaces
		vector <Surface*> surfaces;
		getSurfaces(surfaces);
		//	debugCompare = true;

		// Over-segment each surface
		allSuperPixels.clear();
		size_t tenth = surfaces.size() / 10;
		for(size_t i = 0; i < surfaces.size(); i++) {

			if(i % tenth == 0) { printf("."); fflush(stdout); }
			 // printf("%% surface %d with %d pixels\n", i, surfaces[i]->size());

			// Initialize
			Surface* surface = surfaces[i];
			size_t numPixels = surface->size();

			// Assert that surface has at least 3 points
			//		if(numPixels <= 2) continue;

			// Check if one super pixel would suffice
			size_t maxNumSuperPixels = numPixels / MinSuperPixelSize;
			if(maxNumSuperPixels <= 1) {
				SuperPixel* sp = new SuperPixel (allSuperPixels.size() + 1);
				for(map <Vector2i, size_t>::iterator it = surface->begin(); it != surface->end(); it++)
					sp->insidePixels_.insert(it->first);
				allSuperPixels.push_back(sp);
				continue;
			}

			// Create the superpixels
			size_t minNumSuperPixels = max((size_t)2,numPixels / MaxSuperPixelSize);
			if(maxNumSuperPixels < minNumSuperPixels) throw Exception("Max and min # of superpixels do not make sense");
			size_t numSuperPixels;
			if(maxNumSuperPixels == minNumSuperPixels) numSuperPixels = minNumSuperPixels;
			else numSuperPixels = max((size_t) 1, ((rand() % (maxNumSuperPixels - minNumSuperPixels)) + minNumSuperPixels));
	//		numSuperPixels = 1;
			if(debug) printf("%% nSP: %lu, min: %lu, max: %lu, size: %lu\n", numSuperPixels, minNumSuperPixels, maxNumSuperPixels, numPixels);

			vector <SuperPixel*> superPixels;
			for(size_t i = 0; i < numSuperPixels; i++) {

				// Create the super pixel
				superPixels.push_back(new SuperPixel((allSuperPixels.size())));
				allSuperPixels.push_back(superPixels.back());

				// Pick a random point for its first point and check if it is already picked
				bool alreadyPicked = true;
				while(alreadyPicked) {

					size_t randomPixelIndex = (rand() % numPixels);
					map <Vector2i, size_t>::iterator it = surface->begin();
					advance(it, randomPixelIndex);
					if(it->second != UNDEFINED_LABEL) continue;
					else {
						it->second = superPixels.back()->id_;
						superPixels.back()->edgePixels_.insert(it->first);
						if(debug) printf("%% START pixel: <%d, %d>\n", it->first(0), it->first(1));
						if(debug) printf("%d\t%d\t%lu\n", it->first(0), it->first(1), it->second);
						break;
					}
				}
			}

			// Set pixel labels
			size_t numLabeledPixels = numSuperPixels;
			while(numLabeledPixels < numPixels) {

				// printf("stuck\n");

				// Randomly pick a superpixel
				size_t superPixelIndex = (rand() % numSuperPixels);
				SuperPixel* sp = superPixels[superPixelIndex];
				//			printf("random sp: %d, | %d, %d | edge: %d, in: %d\n", sp->id_, superPixelIndex, numSuperPixels,
				//					sp->edgePixels_.size(), sp->insidePixels_.size());

				// Check if it has an edge pixel and if so, randomly pick it
				// TODO? - Should remove the superpixel from the vector once no more progress can be made
				if(sp->edgePixels_.empty()) {
					// printf("continuing..\n");

					//				for(map<Vector2i,size_t>::iterator it = surface->begin(); it != surface->end(); it++) {
					//					if(it->second == UNDEFINED_LABEL) printf("(%d, %d)\n", it->first(0), it->first(1));
					//				}

					//				printf("\tempty, skipping - %d vs. %d\n", numLabeledPixels, numPixels);
					//				exit(0);
					vector <SuperPixel*>::iterator it = superPixels.begin();
					advance(it, superPixelIndex);
					superPixels.erase(it);
					numSuperPixels--;
					if(numSuperPixels == 0 && (numPixels > numLabeledPixels)) {
						throw Exception("Should not be here!");
					}
					continue;
				}
				size_t randomPixelIndex = (rand() % sp->edgePixels_.size());
				set <Vector2i>::iterator it = sp->edgePixels_.begin();
				advance(it, randomPixelIndex);
				Vector2i p = *it;

				// The directions to search for neighbors
				static const int steps [4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

				// Randomly pick a neighbor to add to the superpixel
				Vector2i neighbors [4];
				size_t numNeighbors = 0;
				for(size_t i = 0; i < 4; i++) {
					Vector2i neighbor = Vector2i(p(0) + steps[i][0], p(1) + steps[i][1]);
					map <Vector2i, size_t>::iterator surf_it = surface->find(neighbor);
					if((surf_it != surface->end()) && (surf_it->second == UNDEFINED_LABEL)) {
						neighbors[numNeighbors++] = neighbor;
					}
				}
				//			for(map<Vector2i,size_t>::iterator it = surface->begin(); it != surface->end(); it++)
				//						printf("(%d, %d)\n", it->first(0), it->first(1));
				if(numNeighbors == 0) {
					map <Vector2i, size_t>::iterator temp = surface->find(p);
					printf("label: %lu; size: %lu\n", temp->second, superPixels.size());
					printf("(%d, %d)\n", p(0), p(1));
				}
				if(numNeighbors == 0) throw Exception("Any edge pixel should have at least one valid neighbor.");
				Vector2i neighbor = neighbors[rand() % numNeighbors];
				(*surface)[neighbor] = sp->id_;

				if(debug) printf("%%\t\t Selected neighbor 1: <%d, %d>\n", neighbor(0), neighbor(1));

				// Add the neighbor and set ITS neighbors (2) to "inside" format if necessary by checking
				// their neighbors (level 3)
				bool isEdgePixel = false;
				for(size_t i = 0; i < 4; i++) {

					// Check if this neighbor of the neighbor is available for any labeling
					Vector2i neighbor2 = Vector2i(neighbor(0) + steps[i][0], neighbor(1) + steps[i][1]);
					map <Vector2i, size_t>::iterator surf_it2 = surface->find(neighbor2);
					if(surf_it2 == surface->end()) {
						if(debug) printf("%%\t\t\t\t Neighbor 2 <%d, %d> is not on surface.\n", neighbor2(0), neighbor2(1));
						continue;
					}

					bool isEdgePixel2 = false;

					// If any pixel is free, let the added pixel be an edge pixel
					if(surf_it2->second == UNDEFINED_LABEL) isEdgePixel = true;

					// If the pixel is not free, check out its neighbors and decide if it should be turned into
					// an "inside" pixel if it already is not.
					else {

						if(debug) printf("%%\t\t\t\t Neighbor 2 <%d, %d> is labeled.\n", neighbor2(0), neighbor2(1));

						// Traverse the neighbors of the neighbor of the added neighbor
						for(size_t j = 0; j < 4; j++) {

							// Check if the neighbor exists in the surface and if it is assigned to any superpixel
							Vector2i neighbor3 = Vector2i(neighbor2(0) + steps[j][0], neighbor2(1) + steps[j][1]);
							map <Vector2i, size_t>::iterator surf_it3 = surface->find(neighbor3);
							if(surf_it3 == surface->end()) {
								if(debug) printf("%%\t\t\t\t\t\t Neighbor 3 <%d, %d> is not on surface.\n", neighbor3(0), neighbor3(1));
							}
							else if((surf_it3->second == UNDEFINED_LABEL)) {
								if(debug) printf("%%\t\t\t\t\t\t Neighbor 3 <%d, %d> is NOT labeled.\n", neighbor3(0), neighbor3(1));
								isEdgePixel2 = true;
								break;
							}
							else {
								if(debug) printf("%%\t\t\t\t\t\t Neighbor 3 <%d, %d> is labeled.\n", neighbor3(0), neighbor3(1));
							}


						}

						// Change the format of the pixel if it already is an edge pixel
						if(!isEdgePixel2) {
							SuperPixel* sp2 = allSuperPixels[surf_it2->second];
							set <Vector2i>::iterator sp_it = sp2->edgePixels_.find(neighbor2);
							if((sp_it != sp2->edgePixels_.end())) {
								if(debug) printf("%% Changing <%d, %d> to inside\n", neighbor2(0), neighbor2(1));
								sp2->edgePixels_.erase(sp_it);
								sp2->insidePixels_.insert(neighbor2);
							}
						}
					}
				}

				// Set the added pixel as an edge or not
				if(isEdgePixel) {
					sp->edgePixels_.insert(neighbor);
					if(debug) printf("%% Added edge pixel for sp %lu: <%d, %d>\n", sp->id_, neighbor(0), neighbor(1));
				}
				else {
					sp->insidePixels_.insert(neighbor);
					if(debug) printf("%% Added inside pixel for sp %lu: <%d, %d>\n", sp->id_, neighbor(0), neighbor(1));
				}
				if(debug) printf("%d\t%d\t%lu\n", neighbor(0), neighbor(1), sp->id_);
				numLabeledPixels++;
			}
		}

		 printf("Oversegmented %lu superpixels.\n", allSuperPixels.size()); fflush(stdout);
	}

	/* ******************************************************************************************* */
	// Draw functions

	void setupCamera () {
		glLoadIdentity();
		glRotatef(cameraProperties.tilt_,1.0,0.0,0.0);
		glRotatef(cameraProperties.pan_,0.0,1.0,0.0);
		Vector3d loc = cameraProperties.loc_;
		glTranslatef(-loc(0), -loc(1), -loc(2));
	}

	size_t* copyHeap (size_t* input, int nRows, int nCols) {

		size_t* result = new size_t [nRows * nCols];
		for(size_t i = 0; i < nRows; i++)
			for(size_t j = 0; j < nCols; j++)
				result[j + nCols * i] = input[j + nCols * i];
		return result;
	}

	void createDigits () {

		size_t zero [9][6] = {{0, 0, 1, 1, 0, 0}, {0, 1, 0, 0, 1, 0}, {1, 0, 0, 0, 0, 1}, {1, 0, 0, 0, 0, 1}, {1, 0, 0, 0, 0, 1}, {1, 0, 0, 0, 0, 1}, {1, 0, 0, 0, 0, 1}, {0, 1, 0, 0, 1, 0}, {0, 0, 1, 1, 0, 0}};
		digits.insert(make_pair(0, copyHeap(zero[0], 9, 6)));

		size_t one [9][6] = {{0, 0, 0, 1, 0, 0}, {0, 0, 1, 1, 0, 0}, {0, 1, 0, 1, 0, 0}, {0, 0, 0, 1, 0, 0}, {0, 0, 0, 1, 0, 0}, {0, 0, 0, 1, 0, 0}, {0, 0, 0, 1, 0, 0}, {0, 0, 0, 1, 0, 0}, {0, 1, 1, 1, 1, 1}};
		digits.insert(make_pair(1, copyHeap(one[0], 9, 6)));

		size_t two [9][6] = {{0, 1, 1, 1, 1, 0}, {1, 0, 0, 0, 0, 1}, {1, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 1, 0}, {0, 0, 1, 1, 0, 0}, {0, 1, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {1, 1, 1, 1, 1, 1}};
		digits.insert(make_pair(2, copyHeap(two[0], 9, 6)));

		size_t three [9][6] = {{1, 1, 1, 1, 1, 1}, {0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 1, 0}, {0, 0, 0, 1, 0, 0}, {0, 0, 1, 1, 1, 0}, {0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 1}, {1, 0, 0, 0, 0, 1}, {0, 1, 1, 1, 1, 0}};
		digits.insert(make_pair(3, copyHeap(three[0], 9, 6)));

		size_t four [9][6] = {{0, 0, 0, 0, 1, 0}, {0, 0, 0, 1, 1, 0}, {0, 0, 1, 0, 1, 0}, {0, 1, 0, 0, 1, 0}, {1, 0, 0, 0, 1, 0}, {1, 0, 0, 0, 1, 0}, {1, 1, 1, 1, 1, 1}, {0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 1, 0}};
		digits.insert(make_pair(4, copyHeap(four[0], 9, 6)));

		size_t five [9][6] = {{1, 1, 1, 1, 1, 1}, {1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {1, 0, 1, 1, 1, 0}, {1, 1, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 1}, {1, 0, 0, 0, 0, 1}, {0, 1, 1, 1, 1, 0}};
		digits.insert(make_pair(5, copyHeap(five[0], 9, 6)));

		size_t six [9][6] = {{0, 0, 1, 1, 1, 0}, {0, 1, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}, {1, 0, 1, 1, 1, 0}, {1, 1, 0, 0, 0, 1}, {1, 0, 0, 0, 0, 1}, {1, 0, 0, 0, 0, 1}, {0, 1, 1, 1, 1, 0}};
		digits.insert(make_pair(6, copyHeap(six[0], 9, 6)));

		size_t seven [9][6] = {{1, 1, 1, 1, 1, 1}, {0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 1, 0}, {0, 0, 0, 1, 0, 0}, {0, 0, 0, 1, 0, 0}, {0, 0, 1, 0, 0, 0}, {0, 0, 1, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}};
		digits.insert(make_pair(7, copyHeap(seven[0], 9, 6)));

		size_t eight [9][6] = {{0, 1, 1, 1, 1, 0}, {1, 0, 0, 0, 0, 1}, {1, 0, 0, 0, 0, 1}, {1, 0, 0, 0, 0, 1}, {0, 1, 1, 1, 1, 0}, {1, 0, 0, 0, 0, 1}, {1, 0, 0, 0, 0, 1}, {1, 0, 0, 0, 0, 1}, {0, 1, 1, 1, 1, 0}};
		digits.insert(make_pair(8, copyHeap(eight[0], 9, 6)));

		size_t nine [9][6] = {{0, 1, 1, 1, 1, 0}, {1, 0, 0, 0, 0, 1}, {1, 0, 0, 0, 0, 1}, {1, 0, 0, 0, 1, 1}, {0, 1, 1, 1, 0, 1}, {0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 1, 0}, {0, 1, 1, 1, 0, 0}};
		digits.insert(make_pair(9, copyHeap(nine[0], 9, 6)));

	}

	void rotate (Vector3d rotation) {
		if(rotation(0) != 0.0) glRotatef(rotation(0), 1.0, 0.0, 0.0);
		if(rotation(1) != 0.0) glRotatef(-rotation(1), 0.0, 1.0, 0.0);
		if(rotation(2) != 0.0) glRotatef(rotation(2), 0.0, 0.0, 1.0);
	}

	void setSurfaceColor() {
		float r = colors[numSurfaces](0) / 255.0;
		float g = colors[numSurfaces](1) / 255.0;
		float b = colors[numSurfaces](2) / 255.0;
		glColor3f(r, g, b);
		numSurfaces++;
	}

	void drawCommand (const DrawCommand& command) {

		if(command.type_.compare("Plane") == 0)
			drawPlane(command.position_, command.rotation_, command.size_);
		else if(command.type_.compare("RectangularPrism") == 0)
			drawRectangularPrism(command.position_, command.rotation_, command.size_);
		else
			throw Exception("Unknown object type to draw from command.");
	}

	void drawRectangularPrism (Vector3d position, Vector3d rotation, Vector3d size) {

		float l = size(0) / 2.0, w = size(1) / 2.0, h = size(2) / 2.0;
		if(!((l > 0.0) && (w > 0.0) && (h > 0.0))) throw Exception("Non-positive size value.");

		setupCamera ();

		glTranslatef(position(0), -position(1), -position(2));
		rotate(rotation);

		glBegin(GL_QUADS);

		setSurfaceColor();
		glVertex3f(l, w, h);							// Front
		glVertex3f(l, -w, h);
		glVertex3f(-l, -w, h);
		glVertex3f(-l, w, h);

		setSurfaceColor();
		glVertex3f(l, w, -h);							// Back
		glVertex3f(l, -w, -h);
		glVertex3f(-l, -w, -h);
		glVertex3f(-l, w, -h);

		setSurfaceColor();
		glVertex3f(-l, w, h);							// Left
		glVertex3f(-l, -w, h);
		glVertex3f(-l, -w, -h);
		glVertex3f(-l, w, -h);

		setSurfaceColor();
		glVertex3f(l, w, h);							// Right
		glVertex3f(l, -w, h);
		glVertex3f(l, -w, -h);
		glVertex3f(l, w, -h);

		setSurfaceColor();
		glVertex3f(l, w, h);							// Top
		glVertex3f(-l, w, h);
		glVertex3f(-l, w, -h);
		glVertex3f(l, w, -h);

		setSurfaceColor();
		glVertex3f(l, -w, h);							// Bottom
		glVertex3f(-l, -w, h);
		glVertex3f(-l, -w, -h);
		glVertex3f(l, -w, -h);

		glEnd();
	}

	void drawPlane (Vector3d position, Vector3d rotation, Vector3d size) {

		float l = size(0) / 2.0, w = size(1) / 2.0;
		if(!((l > 0.0) && (w > 0.0))) throw Exception("Non-positive size value.");

		setupCamera ();
		glTranslatef(position(0), -position(1), -position(2));
		rotate(rotation);

		//	glBegin(GL_POINTS);
		glBegin(GL_QUADS);

		setSurfaceColor();
		glVertex3f(l, w, 0.0);							// Front
		glVertex3f(l, -w, 0.0);
		glVertex3f(-l, -w, 0.0);
		glVertex3f(-l, w, 0.0);

		glEnd();
	}

	void drawSphere (Vector3d position, float radius) {

		if(radius <= 0.0) throw Exception("Non-positive size value.");

		setupCamera ();
		glTranslatef(position(0), -position(1), -position(2));

		setSurfaceColor();
		gluSphere(quadratic, radius, 32, 32);

	}

	void drawDigit (size_t digit, size_t u, size_t v, float** image) {

		if(v < 9) v = 0;
		else v-=9;

		size_t w = cameraProperties.width_;
		size_t* zero = digits[digit];
		for(int i = 8; i >= 0; i--, v++) {
			for(size_t j = 0, uTemp = u; j < 6; j++, uTemp++) {
				if(zero[j+6*i] == 0) continue;
				size_t k = 3 * (uTemp + v * w);
				(*image)[k] = 1;
				(*image)[k+1] = 1;
				(*image)[k+2] = 1;
			}
		}
	}

	void drawNumber (size_t number, size_t u, size_t v, float** image) {

		size_t numDigits = (number == 0) ? 1 : log10(number) + 1;
		size_t temp = number;
		size_t k = 0;
		for(int i = 0; i < numDigits; i++) {
			size_t dividor = pow(10, numDigits-i-1);
			size_t digit = temp / dividor;
			drawDigit(digit, u + k, v, image);
			k += 8;
			temp -= digit * dividor;
		}
	}

	void viewOverSegmentation (vector <SuperPixel*>& superPixels) {

		// Get the pixel data
		float* glImage = new float [3*cameraProperties.width_*cameraProperties.height_];
		initGL(640, 480);
		glReadPixels(0, 0, cameraProperties.width_, cameraProperties.height_, GL_RGB, GL_FLOAT,
				glImage);

		// Set the pixel colors
		vector <SuperPixel*>::iterator sp_it = superPixels.begin();
		size_t colorIndex = 0;
		vector <Vector2i> textLocations;
		for(; sp_it < superPixels.end(); sp_it++, colorIndex+=1) {

			set <Vector2i>::iterator pix_it = (*sp_it)->insidePixels_.begin();
			size_t center_u = 0, center_v = 0;
			for(; pix_it != (*sp_it)->insidePixels_.end(); pix_it++) {

				//			printf("<%d, %d> => <%d, %d, %d>\n", pix_it->x_, pix_it->y_, colors[colorIndex][0], colors[colorIndex][1], colors[colorIndex][2]);
				// Set the pixel value
				size_t k = 3 * ((*pix_it)(0) + (*pix_it)(1) * cameraProperties.width_);
				//			printf("%f\n", glImage[k]);
				if(glImage[k] == 1) continue;
				//			if(pix_it->x_ == 320 && pix_it->y_ == 240) {
				//				drawNumber(1438, pix_it->x_, pix_it->y_, &glImage);
				//				printf("."); fflush(stdout);
				//				continue;
				//			}
				glImage[k] = colors[colorIndex](0) / 255.0;
				glImage[k+1] = colors[colorIndex](1) / 255.0;
				glImage[k+2] = colors[colorIndex](2) / 255.0;

				center_u += (*pix_it)(0);
				center_v += (*pix_it)(1);
			}

			// Draw text at the center
			center_u /= (*sp_it)->insidePixels_.size();
			center_v /= (*sp_it)->insidePixels_.size();
			textLocations.push_back(Vector2i(center_u, center_v));
		}

		for(size_t i = 0; i < textLocations.size(); i++) {
			drawNumber(i, textLocations[i](0), textLocations[i](1), &glImage);
		}

		glDrawPixels(cameraProperties.width_,cameraProperties.height_,GL_RGB,GL_FLOAT,glImage);
		glutSwapBuffers();

	}

	/// Initializes the OpenGL environment
	void initGL (int width, int height) {
		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		glClearDepth(1.0);
		glDepthFunc(GL_LEQUAL);
		glEnable(GL_DEPTH_TEST);
		glShadeModel(GL_SMOOTH);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();

		gluPerspective(cameraProperties.yFov_, cameraProperties.aspectRatio_, zNear, zFar);

		glMatrixMode(GL_MODELVIEW);

		quadratic=gluNewQuadric();          // Create A Pointer To The Quadric Object ( NEW )
		gluQuadricNormals(quadratic, GLU_SMOOTH);   // Create Smooth Normals ( NEW )
		gluQuadricTexture(quadratic, GL_TRUE);
	}


	/* ******************************************************************************************* */
	// IO functions

	Vector3d pixelTo3D (int x, int y) {

		Vector3d point;
		float depth;
		glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
		glLoadIdentity();

		GLdouble modelMatrix[16];
		glGetDoublev(GL_MODELVIEW_MATRIX,modelMatrix);
		GLdouble projMatrix[16];
		glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
		int viewport[4];
		glGetIntegerv(GL_VIEWPORT,viewport);

		gluUnProject(x, y, depth, modelMatrix, projMatrix, viewport, &point(0), &point(1), &point(2));

		return point;
	}

	float randomFromGaussian () {
		float u = ((float) rand()) / RAND_MAX, v = ((float) rand()) / RAND_MAX;
		float random = sqrt(-2.0 * log(u)) * cos(2 * M_PI * v);
		return random;
	}

	float getFocalLength () {

		// Compute the focal length by looking at the 3D coordinates of a random pixel
		int u = 300, v = 200;
		Vector3d loc = pixelTo3D(300, 200);
		return (u - 320) * (-loc(2) / loc(0));
	}

	void saveDrawCommandsFromInput (FILE* file) {

		// Open the file
		std::ifstream ifs(drawFilePath.c_str(), std::ifstream::in);
		if(ifs.fail()) throw Exception("Could not open file.");

		char line [1024];
		while(ifs.good()) {
			ifs.getline(line, 1024);
			if(!ifs.good()) break;
			fprintf(file, "#\t%s\n", line);
		}

		fprintf(file, "#\n");

		// Close the file
		ifs.close();
	}

	void saveDrawCommandsFromCode (FILE* file) {

		// Open this file
		ifstream code ("../src/Simulation.cpp", ifstream::in);
		if(code.fail()) throw Exception("The code could not be opened.");
		fprintf(file, "# The draw commands:\n");

		// Read until the draw commands
		bool started = false;
		char line [1024], temp[1024];
		while(code.good()) {

			// Read the line and check if the list of draws are there
			code.getline(line, 1024);
			if(!code.good()) break;

			if(string(line).find(string(">>> THE ") + "LIST") != string::npos) started = true;
			else if(started && (string(line).find(string("<<< THE ") + "LIST") != string::npos)) break;
			else if(started && (sscanf(line, "%s", &(temp[0])) != 1)) continue;
			else if(started && (string(line).find(string("//")) != string::npos)) {
				printf("line: %s\n", line);
				continue;
			}
			else if(started) fprintf(file, "#\t%s\n", line);
		}

		fprintf(file, "#\n");
	}

	void printPCDHeader (FILE* file, size_t numPoints) {

		fprintf(file, "# Camera properties: \n");
		fprintf(file, "# \tFocal length: %.3f\n", getFocalLength());
		fprintf(file, "# \tResolution: (%lu, %lu)\n", cameraProperties.width_, cameraProperties.height_);
		fprintf(file, "# \tBaseline: %3f\n", cameraProperties.baseline_);
		fprintf(file, "# \tDisparity noise: %3f\n", DISPARITY_NOISE);
		fprintf(file, "#\n");

		if(drawFilePath.empty()) saveDrawCommandsFromCode(file);
		else saveDrawCommandsFromInput(file);

		fprintf(file, "VERSION 0.7\n");
		fprintf(file, "FIELDS x y z\n");
		fprintf(file, "SIZE 4 4 4\n");
		fprintf(file, "TYPE F F F\n");
		fprintf(file, "COUNT 1 1 1\n");
		fprintf(file, "WIDTH 1\n");
		fprintf(file, "HEIGHT %lu\n", numPoints);
		fprintf(file, "VIEWPOINT 0 0 0 1 0 0 0\n");
		fprintf(file, "POINTS %lu\n", numPoints);
		fprintf(file, "DATA ascii\n");

	}

	string createTimeFileName (string prefix) {

		// Get the current time
		time_t rawtime;
		time (&rawtime);
		struct tm* timeinfo = localtime (&rawtime);

		// Create the file name
		char fileName [128];
		strftime(fileName, 128, "%y%m%d-%H%M%S", timeinfo);
		return prefix + fileName;
	}

	void savePCD (vector <SuperPixel*>& superPixels, string fileName, bool shortened) {

		// Assert the baseline is non-negative
		if(cameraProperties.baseline_ <= 0.0) throw Exception("Non-positive baseline.");

		printf("Starting to save to '%s'... ", fileName.c_str()); fflush(stdout);

		// Get the color data
		unsigned char* im = new unsigned char [3 * cameraProperties.width_ * cameraProperties.height_];
		glReadPixels(0,0, cameraProperties.width_, cameraProperties.height_, GL_RGB, GL_UNSIGNED_BYTE, im);

		// Create the pixel data
		vector <PixelData> data;
		vector <SuperPixel*>::iterator sp_it = superPixels.begin();
		float focalLength = getFocalLength();
		for(; sp_it < superPixels.end(); sp_it++) {

			// Traverse the pixels
			set <Vector2i>::iterator pix_it = (*sp_it)->insidePixels_.begin();
			for(; pix_it != (*sp_it)->insidePixels_.end(); pix_it++) {

				// Skip the pixel if it is in the background
				Vector2i pix = (*pix_it);
				size_t k = (pix(1) * cameraProperties.width_ + pix(0)) * 3;
				if((im[k] == 0) && (im[k+1] == 0) && (im[k+2] == 0)) continue;

				// Get the position data - if the 'z' is the far clip, skip
				Vector3d loc = pixelTo3D(pix(0), pix(1));
				if(-loc(2) > zFar) continue;

				// Compute the disparity and add noise to it
				float disparity = (cameraProperties.baseline_ * focalLength) / -loc(2);
				float noiseEffect = randomFromGaussian() * DISPARITY_NOISE;
	//			printf("%f\n", noiseEffect);
				disparity += noiseEffect;

				// Change the location based on the new disparity
				float distZ = -(cameraProperties.baseline_ * focalLength) / disparity;
				float distX = ((int) pix(0) - (int) cameraProperties.width_/2) * distZ / focalLength;
				float distY = ((int) pix(1) - (int) cameraProperties.height_/2) * distZ / focalLength;
				loc = Vector3d(-distX, distY, -distZ);

				// Get the color of the pixel
				Vector3d color (im[k], im[k+1], im[k+2]);

				// Save the pixel
				data.push_back(PixelData(loc, pix, disparity, color, (*sp_it)->id_));
				//			data.push_back(PixelData(loc, pix, disparity, color, 0));
			}
		}

		// Open the file
		if(fileName.empty()) fileName = createTimeFileName("3D-") + ".pcd";
		FILE* file = fopen(fileName.c_str(), "w");
		if(file == NULL) throw Exception("The .pcd file could not be opened");

		// Write the pixel data to the file
		float zMin = 10000000, zMax = -1000000000;
		printPCDHeader(file, data.size());
		for(vector <PixelData>::iterator it = data.begin(); it < data.end(); it++) {
			if(shortened) {
				fprintf(file, "%d\t%d\t%lf\t%lu\n", it->pix_(0), it->pix_(1), it->disparity_, it->label_);
			}
			else {
				fprintf(file, "%lf\t%lf\t%lf\t%d\t%d\t%lf\t%.0lf\t%.0lf\t%.0lf\t%lu\n",
						it->loc_(0), it->loc_(1), it->loc_(2),
						it->pix_(0), it->pix_(1), it->disparity_,
						it->color_(0), it->color_(1), it->color_(2),
						it->label_);
			}
			zMin = fmin(zMin, it->loc_(2));
			zMax = fmax(zMax, it->loc_(2));
		}

		// Close the file
		if(fclose(file) != 0) throw Exception("The file could not be closed successfully");

		printf("PCD file saved.\n"); fflush(stdout);

	}

	void saveDepthImage (string fileName) {

		printf("Saving depth image, this may take time.. "); fflush(stdout);

		// Get the pixel data to skip the background pixels
		unsigned char* im = new unsigned char [3*cameraProperties.width_*cameraProperties.height_];
		glReadPixels(0, 0, cameraProperties.width_, cameraProperties.height_, GL_RGB, GL_UNSIGNED_BYTE,
				im);

		float depths [cameraProperties.width_][cameraProperties.height_];
		float minDepth = zFar + 1, maxDepth = zNear - 1;
		for (size_t y = 0, i = 0; y < cameraProperties.height_; y++) {
			for (size_t x = 0; x < cameraProperties.width_; x++, i+=3) {

				// Skip the pixel if it is in the background
				if((im[i] == 0) && (im[i+1] == 0) && (im[i+2] == 0)) depths[x][y] = zFar;
				else {
					Vector3d point = pixelTo3D(x,y);
					depths[x][y] = -point(2);
				}

			}
		}

		float range = zFar - zNear;
		int imageDepths [cameraProperties.width_][cameraProperties.height_];
		for (size_t x = 0; x < cameraProperties.width_; x++) {
			for (size_t y = 0; y < cameraProperties.height_; y++) {
				imageDepths[x][y] = 255 - (int) (255.0 * ((depths[x][y] - zNear) / range));
			}
		}

		// Create the png image
		png::image< png::rgb_pixel > image(cameraProperties.width_, cameraProperties.height_);
		size_t index = 0;
		for (size_t y = 0; y < image.get_height(); y++)
			for (size_t x = 0; x < image.get_width(); x++)
				image[cameraProperties.height_ - y - 1][x] = png::rgb_pixel(imageDepths[x][y], imageDepths[x][y], imageDepths[x][y]);

		// Write the file
		if(fileName.empty()) fileName = createTimeFileName("depth-") + ".png" ;
		image.write(fileName.c_str());

		printf("Depth image saved.\n");
	}

	void saveImage (string fileName) {

		// Get the pixel data
		unsigned char* glImage = new unsigned char [3*cameraProperties.width_*cameraProperties.height_];
		glReadPixels(0, 0, cameraProperties.width_, cameraProperties.height_, GL_RGB, GL_UNSIGNED_BYTE,
				glImage);

		// Create the png image
		png::image< png::rgb_pixel > image(cameraProperties.width_, cameraProperties.height_);
		size_t index = 0;
		for (size_t y = 0; y < image.get_height(); y++) {
			for (size_t x = 0; x < image.get_width(); x++, index += 3) {
				image[cameraProperties.height_ - y - 1][x] = png::rgb_pixel(glImage[index], glImage[index+1],
						glImage[index+2]);
			}
		}

		// Write the file
		if(fileName.empty()) fileName = createTimeFileName("image-") + ".png";
		image.write(fileName.c_str());

		printf("Color image saved.\n");

	}

	DrawCommand* convertToCommand (const string& line) {

		// Determine if the object is a plane or a rectangular prism by the 5th char
		DrawCommand* com = new DrawCommand();
		size_t index;
		if(line.at(4) == 'P') {
			com->type_ = "Plane";
			index = 10;
		}
		else if(line.at(4) == 'R') {
			com->type_ = "RectangularPrism";
			index = 21;
		}
		else throw Exception("Unknown object type.");

		// Read the parameters
		const char* buf = line.c_str();
		float px, py, pz, rx, ry, rz, sx, sy, sz;
		int temp = sscanf(&(buf[index]), "Vector3d(%f, %f, %f), Vector3d(%f, %f, %f), Vector3d(%f, %f, %f));",
				&px, &py, &pz, &rx, &ry, &rz, &sx, &sy, &sz);
		if(temp != 9) throw Exception("Not all variables read successfully.");

		// Set the command
		com->position_ = Vector3d(px, py, pz);
		com->rotation_ = Vector3d(rx, ry, rz);
		com->size_ = Vector3d(sx, sy, sz);

		return com;
	}

	void applyTransformation (char* line) {

		// Read in the <x,y,z> and <pan,tilt> values
		float x, y, z, pan, tilt;
		int result = sscanf(line, "# %f %f %f %f %f\n", &x, &y, &z, &pan, &tilt);
		if(result != 5) throw Exception("The number of parameters for a transformation is not enough.");

		// Apply the transformation
		cameraProperties.pan_ = pan;
		cameraProperties.tilt_ = tilt;
		cameraProperties.loc_ = Vector3d(x, y, z);
	}

	void createDrawCommands () {

		// Open the file
		std::ifstream ifs(drawFilePath.c_str(), std::ifstream::in);
		if(!ifs.good()) throw Exception("Could not open file.");

		char line [1024];
		while(ifs.good()) {
			ifs.getline(line, 1024);
			if(!ifs.good()) break;
			if(line[0] == '#') {
				applyTransformation(line);
				continue;
			}
			DrawCommand* newCommand = convertToCommand(line);
			if(newCommand == NULL) throw Exception("Bad line in the draw file.");
			inputCommands.insert(newCommand);
		}

		// Close the file
		ifs.close();
	}

}; 
