/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * visibility.h  2017/03/26 10:02
 * TODO:
 *
*/

#pragma once
#ifndef VISIBILITY_H
#define VISIBILITY_H
#include <iostream>
#include <Eigen/core>
#include <igl/matlab/matlabinterface.h>
#include <igl/matlab/MatlabWorkspace.h>
namespace qrcode {
	bool visibility(Engine *engine,Eigen::MatrixXd &V_pxl,Eigen::MatrixXf &Src,Eigen::MatrixXf &Dir,Eigen::MatrixXd &th,Eigen::MatrixXd &th_crv,Eigen::MatrixXd &BW,std::vector<Eigen::MatrixXi> &B_qr,
		Eigen::Matrix4f &mode,double minZ,double t,Eigen::MatrixXd &box,std::vector<std::vector<Eigen::MatrixXd>>&B_md, Eigen::MatrixXd &vis);
	bool visibility2(Engine *engine, Eigen::MatrixXd &V_pxl, Eigen::MatrixXf &Src, Eigen::MatrixXf &Dir, Eigen::MatrixXd &th, Eigen::MatrixXd &th_crv, Eigen::MatrixXd &BW, std::vector<Eigen::MatrixXi> &B_qr,
		Eigen::Matrix4f &mode, double minZ, double t, Eigen::MatrixXd &box, std::vector<std::vector<Eigen::MatrixXd>>&B_md, Eigen::MatrixXd &vis);
}

#endif // !VISIBILITY_H

