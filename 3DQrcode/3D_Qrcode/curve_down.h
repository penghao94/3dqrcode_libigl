/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * curve_down.h  2017/03/12 16:37
 * TODO:
 *
*/
#pragma once
#ifndef CURVEDOWN_H
#define CURVEDOWN_H
#include <Eigen/core>
namespace qrcode {

	//************************************
	// Method:    qrcode::curve_down
	// Returns:   bool
	//
	// @Param Eigen::MatrixXd & V
	// @Param Eigen::MatrixXd & D
	// @Param Eigen::MatrixXd & Src
	// @Param Eigen::MatrixXd & Dir
	// @Param Eigen::MatrixXd & T
	// @Param const int wht_num
	// @Param Eigen::MatrixXd & addT
	// @Param Eigen::MatrixXd & _V
	//************************************
	bool curve_down(Eigen::MatrixXd &V,Eigen::MatrixXd &D,Eigen::MatrixXf &Src, Eigen::MatrixXf &Dir, Eigen::MatrixXd &T, const int wht_num, Eigen::MatrixXd &addT, Eigen::MatrixXd &_V);
}
#endif // !CURVEDOWN_H

