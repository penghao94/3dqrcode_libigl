/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * display.h  2017/03/14 9:25
 * TODO:
 *
*/
#pragma once
#ifndef DISPLAY_H
#define DISPLAY_H
#include <Eigen/core>
namespace qrcode {
	bool display(Eigen::MatrixXd &V1, Eigen::MatrixXi &F1, Eigen::MatrixXd &C1, Eigen::MatrixXi &F_hit,
		Eigen::MatrixXd &V2, Eigen::MatrixXi &F2, Eigen::MatrixXd &C2,
		Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &C);
}

#endif // !DISPLAY_H

