/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * illuminate_map.h  2017/06/14 10:59
 * TODO:
 *
*/
#pragma once
#ifndef ILLUMINATE_MAP_H
#define ILLUMINATE_MAP_H
#include <iostream>
#include <Eigen/core>
#include <igl/embree/EmbreeIntersector.h>
namespace qrcode {
	void illumin_origin(Eigen::MatrixXd &V, Eigen::MatrixXi &F,Eigen::Matrix4f &mode, int num_samples, int distance, Eigen::MatrixXd &P);
	void illuminate_map(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &org, Eigen::MatrixXd &des, Eigen::VectorXd &map);
	void is_light(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::VectorXd &org, Eigen::MatrixXd &des, Eigen::VectorXd &map);
}
#endif // !ILLUMINATE_MAP_H
