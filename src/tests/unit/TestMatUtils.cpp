/**
 * \file TestMatUtils.cpp
 * \brief Unit tests for MatUtils methods
 *
 * \author Andrew Price
 */

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "MatUtils.h"

/**
 * \class TestMatUtils
 * \brief Unit tests for Fastrak class.
 */
class TestMatUtils : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE( TestMatUtils );
	CPPUNIT_TEST(TestReadWrite);
	CPPUNIT_TEST_SUITE_END();

public:
	/**
	 * \fn setUp
	 * \brief Initializes test context for following unit tests for MatUtils methods.
	 */
	void setUp(){}

	/**
	 * \fn tearDown
	 * \brief Cleans up test context for unit tests for MatUtils methods.
	 */
	void tearDown(){}

	/**
	 * \fn TestReadWrite
	 * \brief Verifies
	 */
	void TestReadWrite()
	{
		Eigen::MatrixXf written, read;
		written = Eigen::MatrixXf::Random(10,15);

		saveMatrix("test.bmat", written);
		loadMatrix("test.bmat", read);

		CPPUNIT_ASSERT(read == written);
	}

private:

};

CPPUNIT_TEST_SUITE_REGISTRATION(TestMatUtils);

