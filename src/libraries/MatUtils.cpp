/*
 * MatUtils.cpp
 *
 *  Created on: Apr 3, 2013
 *      Author: arprice
 */


#include "MatUtils.h"

//template<typename MatrixType>
void saveMatrix(const std::string filename, const Eigen::MatrixXf& m)
{
	std::ofstream ofs (filename, std::ios::binary);
	saveMatrix(ofs, m);
	ofs.close();
}

void saveMatrix(std::ofstream& ofs, const Eigen::MatrixXf& m)
{
	long int rows = m.rows();
	long int cols = m.cols();
	//float* d = const_cast<float*>(m.data());
	ofs.write((char *)&rows, sizeof(long int));
	ofs.write((char *)&cols, sizeof(long int));
	//ofs.write((char *)&d, sizeof(Eigen::MatrixXf::Scalar)*m.cols()*m.cols());
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			Eigen::MatrixXf::Scalar val = m(i,j);
			ofs.write((char *)&val, sizeof(Eigen::MatrixXf::Scalar));
		}
	}
}

//template<typename MatrixType>
void loadMatrix(const std::string filename, Eigen::MatrixXf& m)
{
	std::ifstream ifs(filename, std::ios::binary);
	loadMatrix(ifs, m);
	ifs.close();
}

void loadMatrix(std::ifstream& ifs, Eigen::MatrixXf& m)
{
	Eigen::MatrixXf::Index rows, cols;
	ifs.read((char *)&rows, sizeof(rows));
	ifs.read((char *)&cols, sizeof(cols));
	m.resize(rows, cols);

	//float* d = const_cast<float*>(m.data());
	//ifs.read((char *)&d, sizeof(Eigen::MatrixXf::Scalar)*rows*cols);
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			Eigen::MatrixXf::Scalar val;
			ifs.read((char *)&val, sizeof(Eigen::MatrixXf::Scalar));
			m(i,j) = val;
		}
	}
	if (ifs.bad())
		std::cerr << "Bad read." << std::endl;
	//throw std::exception("Error reading matrix");
}
