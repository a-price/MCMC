#ifndef INTERACTIVE_SEGMENTER_H
#define INTERACTIVE_SEGMENTER_H

#include "SegmentationContext.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Common.h"
#include "Graph.h"
#include "IO.h"
#include "OverSegmentation.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>

#include <map>

/* ********************************************************************************************** */
// Colors
const cv::Vec3b segColors[4] = {cv::Vec3b(255,0,0),cv::Vec3b(0,255,0),cv::Vec3b(0,0,255),cv::Vec3b(0,255,255)};


class InteractiveSegmenter
{
	static void mouser(int event, int x, int y, int flags, void* this_)
	{
	  static_cast<InteractiveSegmenter*>(this_)->onMouse(event, x, y, flags, this_);
	}
public:
	InteractiveSegmenter(SegmentationContext sc)
	{
		sc_ = sc;
		cv::namedWindow("Result");
		cv::setMouseCallback("Result", mouser, this);
		cv::imshow("Result", sc_.overSegmented);
	}
	SegmentationContext sc_;

	void onMouse(int event, int x, int y, int flags, void* param)
	{

	    if (event == CV_EVENT_LBUTTONDOWN)
	    {
	    	// Add current superpixel to current S
	    	SuperPixelID sID = (SuperPixelID)sc_.spLookup(y,x);

	    	std::cout << "x=" << x << ", " <<
	    	        	 "y=" << y << ", " <<
	    	        	 "s=" << sID << std::endl;
	    	//StoSP.insert(std::pair<SuperPixelID, int>(sID, currentS));
	    	sc_.StoSP[sc_.currentS].push_back(sID);
	    	sc_.SPtoS.insert(std::pair<int, SuperPixelID>(sc_.currentS, sID));
	    	// Recolor image
	    	repaintSuperPixel(sc_.graph, sc_.overSegmented, sID, sc_.currentS);
	    	cv::imshow("Result", sc_.overSegmented);

	    	std::map<SuperPixelID, SuperPixel*>::const_iterator superPixelIt = sc_.graph.superPixels_.find(sID);
	    	Plane p(*(superPixelIt->second->A_),*(superPixelIt->second->b_));
			gtsam::Vector local = disparityToExplicit(p.density_.mean());
			gtsam::Vector original = (superPixelIt->second->plane_->density_.mean());
			std::cout << "Local Plane Parameters: " << local.matrix().transpose() << std::endl;
			std::cout << "Plane Parameters Disparity: " << original.matrix().transpose() << std::endl;
	    }
	    else if (event == CV_EVENT_RBUTTONDOWN)
	    {
	    	// Prompt for new S
	    	//std::string newS;
	    	//std::cout << "Select new S value: 0=A; 1=B; 2=C; 3=D : \n";
	    	//std::cin >> newS;
	    	//currentS = atoi(newS.c_str());
	    	sc_.currentS++;
	    	sc_.currentS = sc_.currentS % 4;
	    	std::cout << "S index: " << sc_.currentS << std::endl;
	    }
	    else if (event == CV_EVENT_MBUTTONDOWN)
	    {
	    	// go!
	    	cv::imwrite("./temp/superPixels/overSegmented.png", sc_.overSegmented);
	    	computeSuperPlanes();
	    }
	}

	void repaintSuperPixel(const Graph& graph, cv::Mat& segmentedImage, SuperPixelID id, int segID)
	{
		// Find the superpixel index for each pixel
		std::map <SuperPixelID, SuperPixel*>::const_iterator superPixelIt = graph.superPixels_.find(id);//.begin();

		// Set the colors for the pixels
		for(size_t pixelIdx = 0; pixelIdx < superPixelIt->second->A_->rows(); pixelIdx++)
		{
			size_t u = (*superPixelIt->second->A_)(pixelIdx, 0), v = (*superPixelIt->second->A_)(pixelIdx, 1);
			segmentedImage.at<cv::Vec3b>(v,u) = segColors[segID];
		}

	}


	void computeSuperPlanes()
	{
		std::ofstream outfile;
		outfile.open("./temp/superPixels/superVectors.txt");

		for (int i = 0; i < 4; i++)
		{
			std::cout << "S_" << (i) << ":\n";
			//SuperPixel ssp;
			//ssp.A_ = new Matrix();
			//ssp.b_ = new Matrix();
			Eigen::MatrixXd A;// = new Eigen::MatrixXd();
			Eigen::MatrixXd b;// = new Eigen::MatrixXd();
			Eigen::MatrixXd tempA;
			Eigen::MatrixXd tempb;
			A.resize(0,3);
			b.resize(0,1);

			for (int j = 0; j < sc_.StoSP[i].size(); j++)
			{
				//std::cout << "\t" << (j) << ":" << StoSP[i][j] << "\n";

				std::map <SuperPixelID, SuperPixel*>::const_iterator superPixelIt = sc_.graph.superPixels_.find(sc_.StoSP[i][j]);

				gtsam::Vector local = disparityToExplicit(superPixelIt->second->plane_->density_.mean());
				//std::cout << j << " Plane Parameters: " << local.matrix().transpose() << std::endl;

				tempA.resize(A.rows()+superPixelIt->second->A_->rows(),superPixelIt->second->A_->cols());
				tempA << A,*(superPixelIt->second->A_);
				A = tempA;

				tempb.resize(b.rows()+superPixelIt->second->b_->rows(),superPixelIt->second->b_->cols());
				tempb << b,*(superPixelIt->second->b_);
				b = tempb;
			}

			if (A.rows() > 0)
			{
				Plane p(A,b);
				gtsam::Vector local = disparityToExplicit(p.density_.mean());
				std::cout << "Plane Parameters: " << local.matrix().transpose() << std::endl;
				std::cout << "Error: " << p.error_ << "\tNormalized: " << p.error_/A.rows() << std::endl;
				outfile << "0 0 0 " << local[0] << " " << local[1] << " " << local[2] << " " << "\n";

			}

		}
		outfile << "0 0 0 0 0 1 " << std::endl;
	}

};


#endif // INTERACTIVE_SEGMENTER_H

