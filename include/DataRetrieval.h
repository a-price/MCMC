/**
 * @file DataRetrieval.h
 * @author Can Erdogan
 * @date May 02, 2012
 * @brief The main interface with the Kinect device using the modified libfreenect library in the
 * repository. This class has two functionalities. First, it provided disparity information one
 * frame at a time. Second, it transforms disparity values to depths given the conversion table
 * that is optionally provided with the disparity values.
 */

#pragma once

#include "../include/Common.h"

#include "../3rdParty/libfreenect/include/libfreenect.h"
#include "../3rdParty/libfreenect/src/freenect_internal.h"

#include "opencv2/core/core.hpp"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <vector>

class DataRetrieval {
public:

	/// The constructor. It initializes the connection with the Kinect.
	/// @warning It aborts the program if the connection is not made.
	DataRetrieval ();

	/// Returns the disparity and color values for the current frame and optionally, returns the
	/// "disparity to depth" conversion table. Note that the length of the conversionTable is 2048.
	void getFrame (cv::Mat& outputDisparity, cv::Mat& outputColor, uint16_t** conversionTable = NULL);

private:

	static cv::Mat* colorImage;					///< The color image that will be returned with getFrame()
	static cv::Mat* disparityImage;			///< The disparities that will be returned with getFrame()

	static freenect_context *f_ctx;			///< The usb context. Static for the freenect thread.
	static freenect_device* f_dev;			///< The device definition. Static for the freenect thread.
	static uint8_t* rgb_back;						///< The backup buffer for the color. Not sure why needed.

	pthread_t freenect_thread;									///< The freenect thread which retrieves the data
	static pthread_cond_t conditional;					///< The conditional to coordinate the data retrieval
	static pthread_mutex_t conditionalMutex;		///< The mutex for the conditional

	/// The freenect thread function
	static void* freenect_threadfunc (void* arg);

	/// The disparity callback that sets the disparity image
	static void disparity_cb (freenect_device *dev, void *v_depth, uint32_t timestamp,
	    void* disparities = NULL);

	/// The color callback that sets the color image
	static void rgb_cb (freenect_device *dev, void *rgb, uint32_t timestamp);

};
