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
#include<vector>
#include "igl/matlab/matlabinterface.h"
#include <igl/matlab/MatlabWorkspace.h>
#include <igl/Timer.h>
#include <string>
namespace qrcode {
	bool optimization(Engine *engine,Eigen::MatrixXd&D,Eigen::Matrix4f &mode,int wht_num,int mul, Eigen::MatrixXd &V_uncrv,Eigen::MatrixXi &F_qr,Eigen::MatrixXd &C_qr,Eigen::MatrixXi &E_qr,
		Eigen::MatrixXd &H_qr,Eigen::MatrixXf &Src,Eigen::MatrixXf &Dir,std::vector<Eigen::MatrixXd>&th,Eigen::MatrixXd &V_pxl,
		Eigen::MatrixXd &V_rest,Eigen::MatrixXi &F_rest, Eigen::MatrixXi & E_rest, Eigen::MatrixXd &V_fin,Eigen::MatrixXi& F_fin);
	bool vis2gray(Engine *engine,Eigen::MatrixXd &vis, Eigen::MatrixXi &gray);
	bool gray2vis(Engine *engine,Eigen::MatrixXi &gray, Eigen::MatrixXd &vis);
}
#endif // !OPTIMIZATION_H

