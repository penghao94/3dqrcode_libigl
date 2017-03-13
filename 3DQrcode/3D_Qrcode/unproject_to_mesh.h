/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * unproject_to_mesh.h  2017/03/12 10:12
 * TODO:
 *
*/
#pragma once
#ifndef UNPROJECTTOMESH_H_
#define UNPROJECTTOMESH_H_
#include "igl/Hit.h"
#include <Eigen/Core>
#include <functional>
#include "igl/unproject.h"
#include "igl/unproject_ray.h"
#include "igl/ray_mesh_intersect.h"
#include <vector>
#include <iostream>
namespace qrcode {
	//************************************
	// Method:    qrcode::unproject_to_mesh
	// Returns:   bool
	//
	// @Param const Eigen::Vector2f & pos
	// @Param const Eigen::Matrix4f & model
	// @Param const Eigen::Matrix4f & proj
	// @Param const Eigen::Vector4f & viewport
	// @Param const Eigen::MatrixXd & V
	// @Param const Eigen::MatrixXi & F
	// @Param Eigen::Vector3f & s
	// @Param Eigen::Vector3f & dir
	// @Param int & fid
	// @Param Eigen::Vector3f & bc
	// @Param int & t
	//************************************
	bool unproject_to_mesh(const Eigen::Vector2f &pos, const Eigen::Matrix4f &model, const Eigen::Matrix4f &proj, const Eigen::Vector4f &viewport,
		const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, Eigen::Vector3f &s, Eigen::Vector3f &dir, int &fid, Eigen::Vector3f &bc, double &t);
 //************************************
 // Method:    qrcode::unproject_onto_mesh
 // Returns:   bool
 //
 // @Param const Eigen::Vector2f & pos
 // @Param const Eigen::Matrix4f & model
 // @Param const Eigen::Matrix4f & proj
 // @Param const Eigen::Vector4f & viewport
 // @Param const std::function< bool
 // @Param const Eigen::Vector3f &
 // @Param const Eigen::Vector3f &
 // @Param igl::Hit & 
 // @Param > & shoot_ray
 // @Param Eigen::Vector3f & s
 // @Param Eigen::Vector3f & dir
 // @Param int & fid
 // @Param Eigen::Vector3f & bc
 // @Param double & t
 //************************************
 bool unproject_onto_mesh(const Eigen::Vector2f& pos,const Eigen::Matrix4f& model,const Eigen::Matrix4f& proj,const Eigen::Vector4f& viewport,
		const std::function<
		bool(
			const Eigen::Vector3f&,
			const Eigen::Vector3f&,
			igl::Hit  &)
		> & shoot_ray,
		Eigen::Vector3f &s,	Eigen::Vector3f &dir,int & fid,Eigen::Vector3f & bc,double &t);
}

#endif // !UNPROJECTTOMESH_H_

