// This project is a C++ version of qart4j,for more information see https://github.com/dieforfree/qart4j
// 
// Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#pragma once
#ifndef QRCODER_H_
#define QRCODER_H_

#include <string>
#include<sstream>
#include<fstream>
#include <string>

#include <Eigen/core>
#include <igl/png/writePNG.h>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <igl/matlab/matlabinterface.h>

#include "../../bwlabel.h"
#include "../../2D_visible.h"
#include "PixelProperty.h"
#include "Raw.h"
#include "Alpha.h"
#include "RSUtil.h"
#include "Bits.h"
#include "Plan.h"

namespace qrgen {
	typedef std::vector<std::vector<qrgen::Pixel>> MatrixP;

	/*Get minimal version size for data */
	Version * getMinVersion(std::string &text, LEVEL level,Bits &bits);

	/*Encoding QRcode*/
	MatrixP encode(Bits&bits,Version *version, LEVEL level, Mask *mask);

	/*Generate SVG format image*/
	std::string toSvgString(MatrixP &pixels, int border);

	/*Generate Eigen matrix*/
	void toEigenMatrix(MatrixP &pixels,int border, Eigen::MatrixXd &Modules, Eigen::MatrixXd &Functions);

	/*Compute visible area for each module*/
	void setArea(Engine *engine, Eigen::MatrixXd &Modules, Eigen::MatrixXd &Functions, std::vector<std::vector<qrgen::PixelProperty>>& Property);
	/*Generate PNG format image*/
	void writePng(std::string &file, int scale, Eigen::MatrixXd &Modules, Eigen::MatrixXd &Functions);
}
#endif // !QRCODER_H_


