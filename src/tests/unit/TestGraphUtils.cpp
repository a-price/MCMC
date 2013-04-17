/**
 * \file TestGraphUtils.cpp
 * \brief 
 *
 *  \date Apr 3, 2013
 *  \author arprice
 */

//#ifndef TESTGRAPHUTILS_H_
//#define TESTGRAPHUTILS_H_


#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "GraphUtils.h"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

/**
 * \class TestGraphUtils
 * \brief Unit tests for TestGraphUtils
 */
class TestGraphUtils : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE( TestGraphUtils );
	CPPUNIT_TEST(testPMatch);
	CPPUNIT_TEST(testCrossMatch);
	CPPUNIT_TEST(testSerialize);
	CPPUNIT_TEST_SUITE_END();

public:
	/**
	 * \fn setUp
	 * \brief Initializes test context for following unit tests for TestGraphUtils methods.
	 */
	void setUp(){}

	/**
	 * \fn tearDown
	 * \brief Cleans up test context for unit tests for TestGraphUtils methods.
	 */
	void tearDown(){}

	/**
	 * \fn testPMatch
	 * \brief Verifies Bernoulli distribution generating function for model similarity
	 */
	void testPMatch()
	{
		Eigen::VectorXf a, b;
		a = Eigen::VectorXf::Random(50);
		b = a;

		Eigen::VectorXf weights = Eigen::VectorXf::Ones(a.rows());

		// Test Equality
		double p = pMerge(a, b, weights);
		CPPUNIT_ASSERT(p > 0.999999);

		// Test Symmetry
		a = Eigen::VectorXf::Random(50);
		b = Eigen::VectorXf::Random(50);
		CPPUNIT_ASSERT(pMerge(a, b, weights) == pMerge(b, a, weights));
	}

	/**
	 * \fn testCrossMatch
	 * \brief Verifies matching between two sets of models
	 */
	void testCrossMatch()
	{
		int size = 5;
		Eigen::MatrixXf a = Eigen::MatrixXf::Identity(size,size);
		Eigen::MatrixXf b = Eigen::MatrixXf::Identity(size,size);
		Eigen::MatrixXf expected = Eigen::MatrixXf::Zero(size*2,size*2);
		Eigen::MatrixXf result;

		for (int i = 0; i < size; i++)
		{
			expected(i, i + size) = 1;
			expected(i + size, i) = 1;
		}

		//getPairwiseAdjacencyGraph(a,b,result);

		//float err = (result-expected).cwiseAbs().sum();

		//CPPUNIT_ASSERT(err == 0.0);
	}

	/**
	 * \fn testSerialize
	 * \brief Verifies matching between two sets of models
	 */
	void testSerialize()
	{
		SPGraph graphA, graphB;
		graphA = generateSampleGraph();

	    std::ofstream ofs("test.big");
	    std::cout << "Writing graph to file...\n";

	    // save data to archive
	    {
	        boost::archive::text_oarchive oa(ofs);
	        // write class instance to archive
	        oa << graphA;
	        // archive and stream closed when destructors are called
	    }

	    std::ifstream ifs("test.big");
	    {
			boost::archive::text_iarchive ia(ifs);
			// write class instance to archive
			ia >> graphB;
			// archive and stream closed when destructors are called
		}

	    CPPUNIT_ASSERT(graphA.m_edges.size() == graphB.m_edges.size());
	    CPPUNIT_ASSERT(graphA.m_vertices.size() == graphB.m_vertices.size());
	    std::cerr << graphB.m_vertices.size();
	}


private:

};

CPPUNIT_TEST_SUITE_REGISTRATION(TestGraphUtils);


//#endif /* TESTGRAPHUTILS_H_ */
