/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * bwlabel.h  2017/03/22 13:32
 * TODO:
 *
*/
#pragma once
#ifndef BWLABEL_H
#define BWLABEL_H
#include <iostream>
#include <Eigen/core>
#include <igl/matlab/matlabinterface.h>
namespace qrcode {
	// Matlab instance
	bool bwlabel(Engine *engine,Eigen::MatrixXd &BW, int connectivity,Eigen::MatrixXd &L);
	bool bwindex(Engine *engine,Eigen::MatrixXd &V_pxl,Eigen::MatrixXd &L,int scale, std::vector<Eigen::MatrixXd> &V_index);
	bool bwindex(Engine *engine,Eigen::MatrixXd &L, int scale, std::vector<Eigen::MatrixXi>&V_index);
	bool bwindex(Eigen::MatrixXd & L, std::vector<Eigen::MatrixXi>& V_index);
	bool bwindex(Eigen::MatrixXd &L,int scale ,std::vector<Eigen::MatrixXi>&V_index);
	bool upperpoint(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::Matrix4f &mode, int rest_V,int wht_V,int rest_F,int wht_F,Eigen::VectorXi upnt, Eigen::VectorXi &ufct, int &v_num, int &f_num);
}
#endif // !BWLABEL_H

