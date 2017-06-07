/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * qrcodeArea.h  2017/05/08 14:41
 * TODO:
 *
*/
#pragma once
#ifndef QRCODEAREA_H
#define QRCODEAREA_H
#include<iostream>
#include<Eigen/dense>
#include <vector>
#include<igl/matlab/matlabinterface.h>
namespace qrcode {
	double qrcodeArea(Engine *engine, Eigen::VectorXd& v_src,Eigen::MatrixXd &V_qr, Eigen::MatrixXd &V_module, Eigen::MatrixXd &Vs, std::vector<Eigen::MatrixXi> & Fs,Eigen::MatrixXd &V, std::vector<Eigen::MatrixXi> &S,Eigen::MatrixXd &rot);
}
#endif // !QRCODEAREA_H

