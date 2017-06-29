/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * watermark.h  2017/06/21 11:00
 * TODO:
 *
*/
#pragma once
#ifndef WATERMARK_H
#define WATERMARK_H
#include <iostream>
#include <Eigen/core>
#include <igl/matlab/matlabinterface.h>
namespace qrcode {
	void  watermark(Engine *engine, Eigen::MatrixXd&D, Eigen::Matrix4f &mode, int wht_num, int mul, int ext, Eigen::MatrixXd &V_uncrv, Eigen::MatrixXi &F_qr, Eigen::MatrixXd &C_qr, Eigen::MatrixXi &E_qr,
		Eigen::MatrixXd &H_qr, Eigen::MatrixXf &Src, Eigen::MatrixXf &Dir, std::vector<Eigen::MatrixXd>&th, Eigen::MatrixXd &V_pxl,
		Eigen::MatrixXd &V_rest, Eigen::MatrixXi &F_rest, Eigen::MatrixXi & E_rest, Eigen::MatrixXd &V_fin, Eigen::MatrixXi& F_fin);
}
#endif // !WATERMARK_H

