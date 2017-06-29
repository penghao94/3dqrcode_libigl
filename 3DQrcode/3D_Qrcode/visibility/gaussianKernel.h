/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * GaussianKern.h  2017/03/30 9:38
 * TODO:
 *
*/
#pragma once
#ifndef GAUSSIANKERNEL_H
#define GAUSSIANKERNEL_H
#include <Eigen/core>
#include <iostream>
namespace qrcode {
	bool gaussianKernel(int size, Eigen::MatrixXd &G);
}
#endif // !GAUSSIANKERNEL_H

