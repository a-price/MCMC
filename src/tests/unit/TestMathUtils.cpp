/**
 * \file TestMathUtils.cpp
 * \brief 
 *
 *  \date Apr 26, 2013
 *  \author arprice
 */


#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "MathUtils.h"
#include "SampleModelGenerator.h"


/**
 * \class TestMathUtils
 * \brief Unit tests for TestMathUtils
 */
class TestMathUtils : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE( TestMathUtils );
	CPPUNIT_TEST(TestPlaneSVD);
	CPPUNIT_TEST_SUITE_END();

public:
	/**
	 * \fn setUp
	 * \brief Initializes test context for following unit tests for TestMathUtils methods.
	 */
	void setUp(){}

	/**
	 * \fn tearDown
	 * \brief Cleans up test context for unit tests for TestMathUtils methods.
	 */
	void tearDown(){}

	/**
	 * \fn TestPlaneSVD
	 * \brief Simple Test
	 */
	void TestPlaneSVD()
	{
		SampleModelGenerator generator;
		Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
		pose.rotate(Eigen::AngleAxisf(M_PI/4, Eigen::Vector3f::UnitX()));
		pcl::PointCloud<pcl::PointXYZ>::Ptr sample = generator.GenerateSamplePlane(pose, 1, 0.1, 0.01);

		PlaneModel plane = MathUtils::fitPlaneSVD(sample);
		float correlation = fabs(plane.coefficients.topRows(3).dot(pose.rotation().col(2)));

		std::cerr << plane.coefficients.transpose() << std::endl << plane.covariance << std::endl;
		std::cerr << plane.covariance.determinant() << std::endl;

		CPPUNIT_ASSERT(fabs(plane.coefficients.norm() - 1.0) < 0.00001f);
		CPPUNIT_ASSERT(correlation > 0.9);

	}

private:

};

CPPUNIT_TEST_SUITE_REGISTRATION(TestMathUtils);


