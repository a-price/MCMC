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

/**
 * \class TestGraphUtils
 * \brief Unit tests for TestGraphUtils
 */
class TestGraphUtils : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE( TestGraphUtils );
	CPPUNIT_TEST(testPMatch);
	CPPUNIT_TEST(testCrossMatch);
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

		double p = pMerge(a, b, weights);
		CPPUNIT_ASSERT(p > 0.999999);
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

		getPairwiseAdjacencyGraph(a,b,result);

		float err = (result-expected).cwiseAbs().sum();

		CPPUNIT_ASSERT(err == 0.0);
	}


private:

};

CPPUNIT_TEST_SUITE_REGISTRATION(TestGraphUtils);


//#endif /* TESTGRAPHUTILS_H_ */
