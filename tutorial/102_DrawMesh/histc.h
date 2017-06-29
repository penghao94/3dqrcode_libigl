/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * histc.h  2017/05/24 10:37
 * TODO:
 *
*/
#pragma once
#ifndef HISTC_H
#define HISTC_H
#include <iostream>
#include <Eigen/core>
namespace qrcode {
	double histc(Eigen::VectorXd &C);
}
#endif // !HISTC_H
