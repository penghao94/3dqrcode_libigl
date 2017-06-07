/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * visible_mesh.h  2017/05/13 15:49
 * TODO:
 *
*/
#pragma once
#ifndef VISIBLEMESH_H
#define VISIBLEMESH_H
#include <iostream>
#include <Eigen/core>
#include <vector>
#include "mesh.h"
namespace qrcode {
	std::vector<Mesh> visible_mesh(Eigen::MatrixXd & BW,  std::vector<Eigen::MatrixXi> &B_qr, Eigen::MatrixXd &V_pxl, int scale);
}
#endif // !VISIBLEMESH_H
