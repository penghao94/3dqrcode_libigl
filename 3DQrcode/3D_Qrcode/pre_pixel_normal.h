/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * pre_pixel_normal.h  2017/05/13 19:43
 * TODO:
 *
*/
#pragma once
#ifndef PERPIXELNORMAL_H
#define PERPIXELNORMAL_H
#include<iostream>
#include<Eigen/core>
namespace qrcode {
	void pre_black_normal(Eigen::MatrixXd &BW,Eigen::MatrixXf &Src,Eigen::MatrixXf &Dir,Eigen::MatrixXd &th,Eigen::MatrixXd&th_crv, int scale, int num_black,Eigen::MatrixXd &P,Eigen::MatrixXd &N);
	void pre_white_normal(Eigen::MatrixXd &BW, Eigen::MatrixXd &V_pxl, int scale,int num_white, Eigen::MatrixXd &P, Eigen::MatrixXd &N);
	void pre_white_normal2(Eigen::MatrixXd &BW, Eigen::MatrixXd &V_pxl, int scale, int num_white, Eigen::MatrixXd &P, Eigen::MatrixXd &N);
}
#endif
