/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * outBound.h  2017/04/03 21:26
 * TODO:
 *
*/
#pragma once
#ifndef OUTBOUND_H
#define OUTBOUND_H
#include <iostream>
#include <Eigen/core>
namespace qrcode {
	bool outBound(Eigen::MatrixXi &E, int start,int scale, Eigen::MatrixXi &bound);
}
#endif // !OUTBOUND_H
