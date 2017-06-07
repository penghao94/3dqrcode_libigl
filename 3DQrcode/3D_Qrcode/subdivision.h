/*!
* This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
*
* Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
*
* subDivision.h  2017/05/07 20:28
* TODO:
*
*/
#pragma once
#ifndef SUBDIVISION_H
#define SUBDIVISION_H
#include<iostream>
#include<Eigen/core>
#include <Eigen/dense>
#include <vector>
namespace qrcode {
	bool subdivision(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::Matrix4f &mode, Eigen::MatrixXd& Vs, std::vector<Eigen::MatrixXi>& Fs, std::vector<Eigen::MatrixXi> &S);
}
#endif // !SUBDIVISION_H
#pragma once
