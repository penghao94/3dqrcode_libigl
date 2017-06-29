/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * img_to_mesh.h  2017/02/26 16:26
 * TODO: This function is used to convert image into mesh
 *
*/
#ifndef IMGTOMESH_H_
#define IMGTOMESH_H_
#include <iostream>
#include <igl/viewer/Viewer.h>
#include <igl/unproject_in_mesh.h>

namespace qrcode {
	
	//************************************
	// Method:    qrcode::img_to_sep_mesh
	// Returns:   int
	//
	// @Param igl::viewer::Viewer & viewer
	// @Param Eigen::MatrixXd & V
	// @Param Eigen::MatrixXi & F
	// @Param Eigen::MatrixXd & D
	// @Param int scale
	// @Param int acc
	// @Param Eigen::MatrixXi & fid
	// @Param Eigen::MatrixXd & _V
	// @Param Eigen::MatrixXi & _F
	// @Param Eigen::MatrixXd & _C
	// @Param Eigen::MatrixXi & _E
	// @Param Eigen::MatrixXd & _H
	// @Param Eigen::MatrixXf & Src
	// @Param Eigen::MatrixXf & Dir
	// @Param std::vector<Eigen::MatrixXd> & L
	// @Param Eigen::MatrixXd & V_pxl
	//************************************
	int img_to_sep_mesh(
		igl::viewer::Viewer &viewer,
		Eigen::MatrixXd &V,
		Eigen::MatrixXi &F,
		Eigen::MatrixXd &D,
		int scale,
		int acc,
		Eigen::MatrixXi &fid,
		Eigen::MatrixXd &_V,
		Eigen::MatrixXi &_F,
		Eigen::MatrixXd &_C,
		Eigen::MatrixXi &_E,
		Eigen::MatrixXd &_H,
		Eigen::MatrixXf &Src,
		Eigen::MatrixXf &Dir,
		std::vector<Eigen::MatrixXd> &L,
		Eigen::MatrixXd &V_pxl
	);
	
}
#endif // !IMGTOMESH_H_
