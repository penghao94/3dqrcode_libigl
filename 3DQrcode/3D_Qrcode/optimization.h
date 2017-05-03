/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * optimization.h  2017/05/02 22:11
 * TODO:
 *
*/
#pragma once
#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H
#include<iostream>
#include<Eigen/dense>
#include "igl/matlab/matlabinterface.h"
namespace qrcode {
	bool vis2gray(Engine *engine,Eigen::MatrixXd &vis, Eigen::MatrixXi &gray);
	bool gray2vis(Engine *engine,Eigen::MatrixXi &gray, Eigen::MatrixXd &vis);
}
#endif // !OPTIMIZATION_H

