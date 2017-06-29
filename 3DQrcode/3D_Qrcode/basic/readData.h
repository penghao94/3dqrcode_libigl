/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * readData.h  2017/02/26 16:18
 * TODO:
 *
*/
#pragma once
#ifndef READDATA_H_
#define READDATA_H_
#include <iostream>
#include <Eigen/core>
#include <stb_image.h>
#include <igl/matlab/matlabinterface.h>
namespace qrcode {
	
	//************************************
	// Method:    qrcode::readData
	// Returns:   bool
	//
	// @Param: Eigen::MatrixXi & D
	//************************************
	int readData(Eigen::MatrixXi &D);
	//************************************
	// Method:    qrcode::readData
	// Returns:   bool
	//
	// @Param Eigen::MatrixXd & D
	//************************************
	int readData(Eigen::MatrixXd &D);
	//************************************
	// Method:    qrcode::readData
	// Returns:   int
	//
	// @Param Eigen::MatrixXd & D
	// @Param Eigen::MatrixXd & isF
	//************************************
	int readData(Engine *engine,Eigen::MatrixXd &D, Eigen::MatrixXd &isF);

	//************************************
	// Method:    qrcode::readData
	// Returns:   bool
	//
	// @Param: const std::string file
	// @Param: Eigen::MatrixXi & D
	//************************************
	int readData(const std::string file, Eigen::MatrixXi &D);
	//************************************
	// Method:    qrcode::readData
	// Returns:   int
	//
	// @Param const std::string file
	// @Param Eigen::MatrixXd & D
	//************************************
	int readData(const std::string file, Eigen::MatrixXd &D);
	int readData(const std::string file, Engine *engine, Eigen::MatrixXd &D,Eigen::MatrixXd &F);
}
#endif // !READDATA_H_

