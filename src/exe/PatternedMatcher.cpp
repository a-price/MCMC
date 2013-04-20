/**
 * \file PatternedMatcher.cpp
 * \brief 
 *
 *  \date Apr 19, 2013
 *  \author Andrew Price
 */

#include "GraphUtils.h"
#include <Eigen/Core>

#include <boost/archive/text_iarchive.hpp>

#include "SPGraph.h"
//#include "MultiviewProblem.h"

int main(int argc, char** argv)
{
	SPGraph spGraph;

	std::ifstream ifs("test.big");
	{
		boost::archive::text_iarchive ia(ifs);
		// read class instance from archive
		ia >> spGraph;
		// archive and stream closed when destructors are called
	}


}
