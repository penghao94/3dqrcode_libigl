/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * halfedge.h  2017/02/26 23:11
 * TODO: halfedge structure
 *
*/
#pragma once
#ifndef HALFEDGE_H_
#define HALFEDGE_H_
#include <Eigen/core>
namespace qrcode {
	struct eNode 
	{
		int s = 0;
		int d = 0;
		eNode* next;
	};
	class eList{
	public:
		eNode* node;
		eNode* head;
		eList();
		~eList();
		void add( int x, int y);
		void matrix(Eigen::MatrixXi &E);

	};
}
#endif // !HALFEDGE_H_
