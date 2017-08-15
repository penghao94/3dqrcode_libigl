// This file is part of 3DQrcode.
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#pragma once
#ifndef DIRECTION_LIGHT_H
#define DIRECTION_LIGHT_H
#include <iostream>
#include <Eigen/core>
#include <igl/viewer/Viewer.h>
#include <igl/matlab/matlabinterface.h>
namespace qrcode {
	bool direction_light(igl::viewer::Viewer &viewer, Engine *engine,Eigen::Vector4d& position,
		Eigen::MatrixXd &D, Eigen::Matrix4f &mode,float zoom, int wht_num, int mul, int ext,
		Eigen::MatrixXd &V_uncrv, Eigen::MatrixXi &F_qr, Eigen::MatrixXd &C_qr, Eigen::MatrixXi &E_qr,
		Eigen::MatrixXd & H_qr, Eigen::MatrixXf & Src, Eigen::MatrixXf & Dir, std::vector<Eigen::MatrixXd>& th,
		Eigen::MatrixXd & V_pxl, Eigen::MatrixXd & V_rest, Eigen::MatrixXi & F_rest, Eigen::MatrixXi & E_rest,
		Eigen::MatrixXd & V_fin, Eigen::MatrixXi & F_fin);
}
#endif // !DIRECTION_LIGHT_H

