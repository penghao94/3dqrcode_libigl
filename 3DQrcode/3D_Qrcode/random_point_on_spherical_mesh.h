/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * random_point_on_spherical_mesh.h  2017/05/23 22:39
 * TODO:
 *
*/
#pragma once
#ifndef RANDOM_POINT_ON_SPHERICAL_MESH_
#define RANDOM_POINT_ON_SPHERICAL_MESH_
#include <iostream>
#include <Eigen/core>
namespace qrcode {
	void random_points_on_spherical_mesh(const Eigen::Vector3f& origin,const Eigen::MatrixXd &V,const Eigen::MatrixXi&F,int num_samples,int p,Eigen::MatrixXd &S);
	Eigen::Vector3d random_dir();
	double refine(double r);
}
#endif // !RANDOM_POINT_ON_SPHERICAL_MESH_


