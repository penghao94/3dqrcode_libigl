/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * trianglate.h  2017/03/01 16:22
 * TODO:
 *
*/

#pragma once
#ifndef TRANGLATE_H_
#define TRANGLATE_H_
#include <iostream>
#include<Eigen/core>
using namespace std;
namespace qrcode {

	//************************************
	// Method:    qrcode::tranglate
	// Returns:   bool
	//
	// @Param Eigen::MatrixXd & V1
	// @Param Eigen::MatrixXi & E1
	// @Param Eigen::MatrixXd & V2
	// @Param Eigen::MatrixXi & E2
	// @Param Eigen::MatrixXd & H
	// @Param Eigen::MatrixXd & _V
	// @Param Eigen::MatrixXi & _F
	//************************************
	bool tranglate(Eigen::MatrixXd &V1,
		Eigen::MatrixXi &E1,
		Eigen::MatrixXd &V2,
		Eigen::MatrixXi &E2,
		Eigen::MatrixXd &H,
		Eigen::MatrixXd &_V,
		Eigen::MatrixXi &_F);
	//************************************
	// Method:    qrcode::tranglate
	// Returns:   bool
	//
	// @Param Eigen::MatrixXd & V1
	// @Param Eigen::MatrixXi & E1
	// @Param Eigen::MatrixXd & V2
	// @Param Eigen::MatrixXi & E2
	// @Param Eigen::MatrixXd & H
	// @Param Eigen::MatrixXi & _F
	//************************************
	bool tranglate(Eigen::MatrixXd &V1,
		Eigen::MatrixXi &E1,
		Eigen::MatrixXd &V2,
		Eigen::MatrixXi &E2,
		Eigen::MatrixXd &H,
		Eigen::Matrix4f &mode,
		Eigen::MatrixXi &F);

}
#endif // !TRANGLATE_H_
