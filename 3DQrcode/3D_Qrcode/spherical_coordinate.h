 /*!
  * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
  *
  * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
  *
  * spherical_coordinate.h  2017/05/12 17:25
  * TODO:
  *
 */
#pragma once
#ifndef SPHERICALCOORDINATE_H
#define SPHERICALCOORDINATE_H
#include<iostream>
#include<Eigen/Core>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
namespace qrcode {
	Eigen::MatrixXd rot(const Eigen::VectorXd &before, const Eigen::VectorXd &after);
	double Box(const Eigen::Vector3f&origin, const Eigen::MatrixXd &V, const Eigen::MatrixXd &rot,Eigen::MatrixXd &Box);
}
#endif // !SPHERICALCOORDINATE_H
