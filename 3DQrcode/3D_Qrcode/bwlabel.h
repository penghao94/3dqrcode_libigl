/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * bwlabel.h  2017/03/22 13:32
 * TODO:
 *
*/
#pragma once
#ifndef BWLABEL_H
#define BWLABEL_H
#include <iostream>
#include <Eigen/core>
#include <igl/matlab/matlabinterface.h>
namespace qrcode {
	// Matlab instance
	bool bwlabel(Eigen::MatrixXd &BW, int connectivity,Eigen::MatrixXd &L);
	bool bwindex(Eigen::MatrixXd &L, std::vector<Eigen::Matrix3d> &V_index);
}
#endif // !BWLABEL_H

