/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * radom_dir.h  2017/05/12 21:17
 * TODO:
 *
*/
#pragma once
#ifndef RANDOMDIR_H
#define RANDOMDIR_H
#include<iostream>
#include<Eigen/core>
#include<vector>
#include "../mesh.h"
namespace qrcode {
	std::vector<Eigen::Vector3f> random_dir(const Eigen::Vector3f &origin,const Mesh &mesh, const Eigen::MatrixXd &box,const int sample);
	Eigen::MatrixXd random_dir(const int num_samples);
}
#endif // !RANDOMDIR_H
