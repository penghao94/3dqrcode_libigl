/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * cut_plane.h  2017/04/13 10:12
 * TODO: call CGAL AABB tree to cut plane
 *
*/
#pragma once
#ifndef CUTPLANE_H
#define CUTPLANE_H
#include<Eigen/core>
#include<iostream>
#include<vector>
#include "igl/matlab/matlabinterface.h"
#include <CGAL/AABB_intersections.h> 
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/AABB_halfedge_graph_segment_primitive.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>

namespace qrcode {
	bool cut_plane(Engine *engine,Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::Matrix4f &mode,int layer, std::vector<std::vector<Eigen::MatrixXd>> &B,double &minZ,double &t,Eigen::MatrixXd &Box);

}
#endif // !CUTPLANRE_H

