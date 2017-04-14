/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * 2D_visible.h  2017/04/06 21:05
 * TODO: this function use CGAL 2D_visibility to find lighting region
 *
*/
#pragma once
#ifndef VISIBLE_H
#define VISIBLE_H
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Triangular_expansion_visibility_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <iostream>
#include <vector>
#include <Eigen/core>
namespace qrcode {
	
	//************************************
	// Method:    qrcode::lightRegion
	// Returns:   bool
	//
	// @Param Eigen::RowVector2i & query_point
	// @Param Eigen::MatrixXi & E
	// @Param int scale
	// @Param int col
	// @Param std::vector<Eigen::RowVector2i> & R
	//************************************
	bool lightRegion(Eigen::RowVector2d &query_point, Eigen::MatrixXi &E, int scale, int col, std::vector<Eigen::Vector2d> &R);
	
	//************************************
	// Method:    qrcode::lightRegion
	// Returns:   bool
	//
	// @Param Eigen::Vector3d & query_point
	// @Param Eigen::MatrixXd & V
	// @Param Eigen::Matrix4f & mode
	// @Param double minZ
	// @Param double t
	// @Param std::vector<std::vector<Eigen::MatrixXd>> & B
	// @Param std::vector<Eigen::MatrixXd> & R
	//************************************
	bool lightRegion(Eigen::Vector3d &query_point,Eigen::Matrix4f& mode,double minZ,double t,std::vector<std::vector<Eigen::MatrixXd>>&B, std::vector<Eigen::MatrixXd>&R);
}
#endif // !2DVISIBLE_H

