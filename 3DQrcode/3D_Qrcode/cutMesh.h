/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * cutMesh.h  2017/02/26 17:12
 * TODO:
 *
*/
#pragma once
#ifndef CUTMESH_H_
#define CUTMESH_H_
#include <iostream>
#include <Eigen/core>
#include "halfedge.h"
#include <igl/unique.h>
namespace qrcode {
	//************************************
	// Method:    qrcode::cutMesh
	// Returns:   bool
	//
	// @Param Eigen::MatrixXd & V
	// @Param Eigen::MatrixXi & F
	// @Param Eigen::MatrixXi & fid
	// @Param Eigen::MatrixXd & _V
	// @Param Eigen::MatrixXi & _F
	//************************************
	bool cutMesh(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXi &fid, Eigen::MatrixXd &_V, Eigen::MatrixXi &_F);
	
	//************************************
	// Method:    qrcode::cutMesh
	// Returns:   bool
	//
	// @Param Eigen::MatrixXd & V
	// @Param Eigen::MatrixXi & F
	// @Param Eigen::MatrixXi & fid
	// @Param Eigen::MatrixXd & _V
	// @Param Eigen::MatrixXi &
	// @Param Eigen::MatrixXi & Ee
	//************************************
	bool cutMesh(Eigen::MatrixXd &V,Eigen::MatrixXi &F,Eigen::MatrixXi &fid,Eigen::MatrixXd &_V,Eigen::MatrixXi &_F,Eigen::MatrixXi &Ee);
}
#endif // !CULMESH_H_  

