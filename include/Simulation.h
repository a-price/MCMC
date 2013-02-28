/**
 * @file Simulation.h
 * @author Can Erdogan
 * @date Oct 20, 2012
 * @brief This file contains the helper functions for the simulation executable.
 * The helpers functions can be divided into two categories: draw commands
 * and oversegmentation. Note that the oversegmentation module randomly creates
 * superpixels to simulate a well textured environment.
 */

#pragma once

#include "Common.h"

#include <png++/png.hpp>

#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <err.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <set>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace simulation {

	/* ******************************************************************************************* */
	// Helper structures

	/// Compare function for Eigen::Vector2i
	struct Vector2iComparator {
		bool operator () (const Vector2i& p1, const Vector2i& p2) const {
			return ((p1(0) < p2(0)) || ((p1(0) == p2(0)) && (p1(1) < p2(1))));
		}
	};

	/// Compare function for Eigen::Vector3d
	struct Vector3dComparator {
		bool operator () (const Vector3d& p1, const Vector3d& p2) const {
			return (p1(0) < p2(0)) || ((p1(0) == p2(0)) && (p1(1) < p2(1)))
					|| ((p1(0) == p2(0)) && (p1(1) == p2(1)) && (p1(2) == p2(2)));
		}
	};

	/// The pixel data to be written for the .pcd file
	struct PixelData {

		Vector3d loc_;																						///< The 3D location of the pixel
		Vector2i pix_;																						///< The pixel coordinates
		double disparity_;																			///< The disparity at the given pixel
		Vector3d color_;																					///< The color of the pixel
		size_t label_;																					///< The label associated with this pixel

		/// The constructor with the input field values
		PixelData (const Vector3d& loc, Vector2i pix, double disparity, const Vector3d& color,
				size_t label) : loc_(loc), pix_(pix), disparity_(disparity), color_(color),
				label_(label) {}
	};

	/// The properties of the OpenGL camera
	struct CameraProperties {

		Vector3d loc_;																						///< The location of the camera
		float pan_;																							///< The pan value in degrees
		float tilt_;																						///< The tilt value in degrees
		float yFov_;																						///< The vertical field of view in degrees
		float aspectRatio_;																			///< The aspect ratio of the camera
		float baseline_;																				///< The baseline for a camera rig
		size_t width_;																					///< The width of the image
		size_t height_;																					///< The height of the image

		/// The default constructor
		CameraProperties () : pan_(0.0), tilt_(0.0), yFov_(0.0), aspectRatio_(0.0), baseline_(0.0),
				width_(640), height_(480) {}

		/// The constructor with the field parameters
		CameraProperties(Vector3d loc, float width, float height, float baseline, float yFov,
				float aspectRatio) : loc_(loc), width_(width), height_(height),
				baseline_(baseline), yFov_(yFov), aspectRatio_(aspectRatio), pan_(0.0), tilt_(0.0) {}

	} cameraProperties;

	/// The definition of a super pixel, i.e. a set of connected pixels
	struct SuperPixel {

		size_t id_;																											///< Super pixel ID
		std::set <Vector2i, Vector2iComparator> insidePixels_;							///< List of pixels inside
		std::set <Vector2i, Vector2iComparator> edgePixels_;								///< List of pixels at the edge

		/// The constructor
		SuperPixel (size_t id) : id_(id) {}
	};

	/// The definition of a surface
	struct Surface : std::map <Vector2i, size_t, Vector2iComparator> {
		Vector3d color_;			///< The color of the surface
	};

	/// The definition of a draw command
	struct DrawCommand {

		string type_;						///< Can be "Plane" or "RectangularPrism".
		Vector3d position_;				///< The position of the object
		Vector3d rotation_;				///< The rotation of the object
		Vector3d size_;						///< The size of the object (for planes, last dimension does not matter)

		/// The default constructor
		DrawCommand () {}

		/// The constructor
		DrawCommand (string type, Vector3d position, Vector3d rotation, Vector3d size) :
				type_(type), position_(position), rotation_(rotation), size_(size) {
		}
	};


	/* ******************************************************************************************* */
	// Global variables

	const float DISPARITY_NOISE = 0.05;										///< The noise on the disparity
	const float KINECT_FOV_Y = 43.0;											///< The vertical Kinect field of view
	const float KINECT_ASPECT_RATIO = 4.0 / 3.0;					///< The Kinect aspect ratio
	const float KINECT_IR_BASELINE = 0.10;								///< The baseline
	const float zFar = 2000.0;														///< The far clip plane depth
	const float zNear = 0.1;															///< The near clip plane depth
	const size_t MinSuperPixelSize = 800;									///< The minimum superpixel size
	const size_t MaxSuperPixelSize = 3000; 								///< The maximum superpixel size

	int window;																						///< The OpenGL window
	size_t numSurfaces;																		///< The number of drawn surfaces
	GLUquadricObj* quadratic;															///< Object to draw spheres
	string requestedFileName;															///< File name for '3'.
	string drawFilePath;																	///< The path for the scene to be drawn
	const bool debug = false;															///< Debugging parameter

	static const size_t UNDEFINED_LABEL = numeric_limits <size_t>::max();		///< Undefined label identifier
	set <DrawCommand*> inputCommands;											///< The set of draw commands from the input
	vector <Vector3i> colors;															///< The set of possible colors
	map <size_t, size_t*> digits;													///< The pixel locations of digis

	/* ******************************************************************************************* */
	// Oversegmentation

	/// Gets the surfaces from the RGBD image
	void getSurfaces (vector <Surface*>& surfaces);

	/// Oversegments the RGBD image
	void oversegment (vector <SuperPixel*>& allSuperPixels);
	
	/* ******************************************************************************************* */
	// Draw functions

	/// Setups the OpenGL with the current camera properties
	void setupCamera ();

	/// Makes a copy of the size_t 2D matrix in the heap
	size_t* copyHeap (size_t* input, int nRows, int nCols);

	/// Create the pixel locations for the digits to be drawn
	void createDigits ();

	/// Rotates the OpenGL environment with the given rotation
	void rotate (Vector3d rotation);

	/// Sets the surface color
	void setSurfaceColor();

	/// Calls the appropriate draw function given the requested command
	void drawCommand (const DrawCommand& command);

	/// Draws a rectangular prism with the given position, rotation, size and color.
	void drawRectangularPrism (Vector3d position, Vector3d rotation, Vector3d size);

	/// Draws a rectangular prism with the given position, rotation, size and color.
	void drawPlane (Vector3d position, Vector3d rotation, Vector3d size);

	/// Draws a sphere at the given position with the given radius
	void drawSphere (Vector3d position, float radius);

	/// Draws a digit to the image at the given (u,v) pixel
	void drawDigit (size_t digit, size_t u, size_t v, float** image);

	/// Draws a number to the image at the given (u,v) pixel
	void drawNumber (size_t number, size_t u, size_t v, float** image);

	/// Visualizes an over-segmentation
	void viewOverSegmentation (vector <SuperPixel*>& superPixels);

	/// Initializes the OpenGL environment
	void initGL (int width, int height);

	/* ******************************************************************************************* */
	// IO functions

	/// Returns the 3D coordinates of a pixel
	Vector3d pixelTo3D (int x, int y);

	/// Box-Muller method to generate a random value from a Normal (0,1) distribution (see wikipedia).
	float randomFromGaussian ();

	/// Compute the focal length by looking at the 3D coordinates of a random pixel
	float getFocalLength ();

	// Saves the draw commands in the draw() function from the input file
	void saveDrawCommandsFromInput (FILE* file);

	// Saves the draw commands in the draw() function from the code
	void saveDrawCommandsFromCode (FILE* file);

	/// Prints the PCD file header
	void printPCDHeader (FILE* file, size_t numPoints);

	/// Returns a file name with the current time
	string createTimeFileName (string prefix = string());

	/// Saves the scene as a pcd file
	void savePCD (vector <SuperPixel*>& superPixels, string fileName = string(), bool shortened = false);

	/// Saves an image where the pixels have normalized depth values.
	void saveDepthImage (string fileName = string());

	/// Saves the current OpenGL render as an image named with the current date and time.
	void saveImage (string fileName = string());

	/// Converts a string to a draw command if possible
	DrawCommand* convertToCommand (const string& line);

	/// Applies the transformation outlined by the string from the draw file
	void applyTransformation (char* line);

	/// Creates draw commands from the given draw file
	void createDrawCommands ();

};
