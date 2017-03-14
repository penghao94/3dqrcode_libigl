/*
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Lorrain <lorrain0407@gmail.com>
 *
 * ambientSetting.h  2017/03/08 13:53
 * TODO: This function is used to set ambient for simulation
 *
*/

#pragma once
#ifndef  EMBIENTSETTING_H
#define  EMBIENTSETTING_H
#include <Eigen/core>
#include <igl/viewer/Viewer.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_corner_normals.h>
#include <igl/embree/ambient_occlusion.h>

namespace qrcode {
	
	//************************************
	// Method:    ambientSetting
	// FullName:  qrcode::ambientSetting
	// Access:    public 
	// Returns:   bool
	// Qualifier:
	// Parameter: igl::viewer::Viewer & viewer
	// Parameter: Eigen::MatrixXd & V
	// Parameter: Eigen::MatrixXi & F
	// Parameter: float & lighting_factor
	//************************************
	bool ambientSetting(igl::viewer::Viewer &viewer,
		Eigen::MatrixXd &V,
		Eigen::MatrixXi &F,
		float &lighting_factor);

	//************************************
	// Method:    ambientSetting
	// FullName:  qrcode::ambientSetting
	// Access:    public 
	// Returns:   bool
	// Qualifier:
	// Parameter: igl::viewer::Viewer & viewer
	// Parameter: Eigen::MatrixXd & V
	// Parameter: Eigen::MatrixXi & F
	// Parameter: float & lighting_factor
	// Parameter: double r
	// Parameter: double g
	// Parameter: double b
	//************************************
	bool ambientSetting(igl::viewer::Viewer &viewer,
		Eigen::MatrixXd &V,
		Eigen::MatrixXi &F,
		float &lighting_factor,
		double &R,
		double &G,
		double &B);

}
#endif // !EMBIENTSETTING_H

