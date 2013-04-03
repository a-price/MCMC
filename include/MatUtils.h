/*
 * MatUtils.h
 *
 *  Created on: Apr 3, 2013
 *      Author: arprice
 */

#ifndef MATUTILS_H_
#define MATUTILS_H_

#include <iostream>
#include <string>
#include <fstream>

#include <Eigen/Core>

//template<typename MatrixType>
void saveMatrix(const std::string filename, const Eigen::MatrixXf& m);
void saveMatrix(std::ofstream& ofs, const Eigen::MatrixXf& m);

//template<typename MatrixType>
void loadMatrix(const std::string filename, Eigen::MatrixXf& m);
void loadMatrix(std::ifstream& ifs, Eigen::MatrixXf& m);


#endif /* MATUTILS_H_ */
