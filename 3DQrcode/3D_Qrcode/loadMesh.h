/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * loadMesh.h  2017/02/26 16:39
 * TODO: Load Mesh
 *
*/
#pragma once
#ifndef LOADMESH_H_
#define LOADMESH_H_
#include <iostream>
#include<igl/viewer/Viewer.h>
#include<Eigen/core>
namespace qrcode {
	//************************************
	// Method:    qrcode::loadMesh
	// Returns:   bool
	//
	// @Param igl::viewer::Viewer & viewer
	// @Param Eigen::MatrixXd & V
	// @Param Eigen::MatrixXi & F
	//************************************
	bool loadMesh(igl::viewer::Viewer &viewer, Eigen::MatrixXd &V, Eigen::MatrixXi &F);
}
#endif // !LOADMESH_H_
