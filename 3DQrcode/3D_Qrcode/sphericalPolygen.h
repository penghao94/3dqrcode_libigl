/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * shericalPolygen.h  2017/04/19 21:58
 * TODO:
 *
*/
#pragma once
#ifndef SPHERICALPOLYGEN_H
#define SPHERICALPOLYGEN_H
#include<iostream>
#include<vector>
#include <Eigen/dense>
#include <igl/matlab/matlabinterface.h>
namespace qrcode {
	double multiIntersection(Engine *engine,Eigen::MatrixXd &V_qr,std::vector<Eigen::MatrixXd> &V_md,std::vector<Eigen::MatrixXd>&V_flag,Eigen::MatrixXd &rot,Eigen::MatrixXd &dir);
	double qrArea(Engine *engine, Eigen::MatrixXd &V_qr, Eigen::MatrixXd &rot);
	void rotMatrix(Eigen::VectorXd &before, Eigen::VectorXd &after, Eigen::MatrixXd &rot);
}
#endif // !SPHERICALPOLYGEN_H

#include<iostream>
#include<vector>
#include <Eigen/dense>
