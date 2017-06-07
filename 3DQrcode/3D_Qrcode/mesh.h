/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * mesh.h  2017/05/11 23:02
 * TODO: Mesh type
 *
*/
#pragma once
#ifndef MESH_H
#define MESH_H
#include<Eigen/Core>
namespace qrcode {
	struct Mesh 
	{
		Eigen::MatrixXd V;// Mesh vertexes
		Eigen::MatrixXi F;// Mesh Facets
	};
}
#endif // !MESH_H

