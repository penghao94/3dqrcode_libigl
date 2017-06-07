/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * ambient_occlusion.h  2017/05/11 22:37
 * TODO:Adjust libigl ambient occlusion algorithm to suit qrcode feature
 *
*/
#pragma once
#ifndef AMBIENTOCCLUSION_H
#define AMBIENTOCCLUSION_H
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <functional>
#include <algorithm>
#include <igl/ambient_occlusion.h>
#include <igl/embree/ambient_occlusion.h>
#include "mesh.h"
namespace qrcode {
	// Method:    qrcode::ambient_occlusion
	// Returns:   void
	// overwrite igl/embree/ambient_occlusion
	//
	//Inputs:
	// @Param Eigen::MatrixXd & V
	// @Param Eigen::MatrixXi & F
	// @Param Eigen::MatrixXd & P
	// @Param Eigen::MatrixXd & N
	// @Param int num_samples
	//Outputs:
	// @Param Eigen::VectorXd & S
	void ambient_occlusion(
		const Eigen::MatrixXd &V,
		const Eigen::MatrixXi &F,
		const Eigen::MatrixXd &P,
		const Eigen::MatrixXd &N,
		const int num_samples,
		Eigen::VectorXd &S);
	void ambient_occlusion(
		const igl::embree::EmbreeIntersector &ei,
		const Eigen::MatrixXd &P,
		const Eigen::MatrixXd &N,
		const int num_samples,
		Eigen::VectorXd &S
	);
	void ambient_occlusion(
		const std::function<
		bool(
			const Eigen::Vector3f&,
			const Eigen::Vector3f&)
		> & shoot_ray,
		const Eigen::MatrixXd & P,
		const Eigen::MatrixXd & N,
		const int num_samples,
		Eigen::VectorXd & S
	);
	// Method:    qrcode::ambient_occlusion
	// Returns:   void
	// Compute ambient occlusion per given point with region constraint
	//
	//Inputs:
	// @Param Eigen::MatrixXd & V
	// @Param Eigen::MatrixXi & F
	// @Param Eigen::MatrixXd & P
	// @Param Eigen::MatrixXd & N
	// @Param std::vector<Mesh> & M
	// @Param int num_samples
	//Outputs:
	// @Param Eigen::VectorXd & S
	void ambient_occlusion(
		const Eigen::MatrixXd &V,
		const Eigen::MatrixXi &F,
		const Eigen::MatrixXd &P,
		const Eigen::MatrixXd &N,
		const std::vector<qrcode::Mesh> &M,
		Eigen::VectorXd &S
	);
	void ambient_occlusion(
		const igl::embree::EmbreeIntersector &ei,
		const Eigen::MatrixXd& P,
		const Eigen::MatrixXd& N,
		const std::vector<qrcode::Mesh>& M,
		 Eigen::VectorXd &S
		);
	void ambient_occlusion(
		const std::function<
		bool(
			const Eigen::Vector3f&,
			const Eigen::Vector3f&)
		> & shoot_ray,
		const Eigen::MatrixXd &P,
		const Eigen::MatrixXd &N,
		const std::vector<qrcode::Mesh> &mesh,
		Eigen::VectorXd &S
	);
}

#endif // !AMBIENTOCCLUSION_H

