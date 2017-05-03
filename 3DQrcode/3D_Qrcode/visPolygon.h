#pragma once
/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * visPolygon.h  2017/04/27 20:25
 * TODO: this function are used to compute visible polygon for each module and optimized by openMP
 *
*/
#ifndef VISPOLYGON_H
#define VISPOLYGON_H
#include<omp.h>
#include<iostream>
#include <vector>
#include <Eigen/core>
//CGAL
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Triangular_expansion_visibility_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
//Boost
#include <deque>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/foreach.hpp>

namespace qrcode {
	// Define the CGAL kernel and arrangement  
	typedef CGAL::Exact_predicates_exact_constructions_kernel       Kernel;
	typedef Kernel::FT												FT;
	typedef Kernel::Point_2											Point_2;
	typedef Kernel::Segment_2                                       Segment_2;
	typedef CGAL::Arr_segment_traits_2<Kernel>                      Traits_2;
	typedef CGAL::Arrangement_2<Traits_2>                           Arrangement_2;
	typedef Arrangement_2::Halfedge_const_handle                    Halfedge_const_handle;
	typedef Arrangement_2::Face_handle                              Face_handle;
	typedef CGAL::Triangular_expansion_visibility_2<Arrangement_2, CGAL::Tag_true>  TEV;
	//define Boost spherical coordinate system
	namespace bg = boost::geometry;
	typedef boost::geometry::model::d2::point_xy<double> Point_c;
	typedef	bg::model::point<double, 2, bg::cs::spherical_equatorial<bg::radian>> Point_s;
	typedef bg::model::polygon<Point_c> Polygon_c;
	typedef bg::model::polygon<Point_s> Polygon_s;
	bool qrPolygon(Eigen::RowVector2d &query_point, Eigen::MatrixXi &E, int scale, int col,Eigen::MatrixXd &V_pxl,Eigen::VectorXd &V_src,Eigen::MatrixXd &rot,Polygon_c &qrpoly);
	bool slPolygon(Eigen::VectorXd &src, Eigen::VectorXd &des, Eigen::Matrix4f & mode, const double minZ,const double t, const int i,Eigen::MatrixXd &Box,
		std::vector<Eigen::MatrixXd>& B, Eigen::MatrixXd &rot, Polygon_c &slpoly);
	bool spi2sphere(Eigen::VectorXd &dir, Eigen::VectorXd &a,Polygon_c &polygon);
	bool spi2sphere(Eigen::VectorXd &dir, Eigen::VectorXd &a,const double flag, Polygon_c &polygon);
}
#endif // !VISPOLYGON_H

