/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * ray_intersect.h  2017/03/28 15:49
 * TODO:
 *
*/
#pragma once
#ifndef RAY_INTERSECT_H
#define RAY_INTERSECT_H
#include "igl/Hit.h"
#include <Eigen/Core>
#include <vector>
namespace qrcode
{
	// Shoot a ray against a mesh (V,F) and collect all hits.
	//
	// Inputs:
	//   source  3-vector origin of ray
	//   dir  3-vector direction of ray
	//   V  #V by 3 list of mesh vertex positions
	//   F  #F by 3 list of mesh face indices into V
	// Outputs:
	//    hits  **sorted** list of hits
	// Returns true if there were any hits (hits.size() > 0)
	//
		bool ray_mesh_intersect(
			const Eigen::VectorXd & source,
			const Eigen::VectorXd & dir,
			const Eigen::MatrixXd & V,
			const Eigen::MatrixXi & F,
			std::vector<igl::Hit> & hits);
	// Outputs:
	//   hit  first hit, set only if it exists
	// Returns true if there was a hit
	
		 bool ray_mesh_intersect(
			 const Eigen::VectorXd & source,
			 const Eigen::VectorXd & dir,
			 const Eigen::MatrixXd & V,
			 const Eigen::MatrixXi & F,
			igl::Hit & hit);
}
#endif

