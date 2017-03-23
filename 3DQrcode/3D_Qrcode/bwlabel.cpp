/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * bwlabel.cpp  2017/03/22 13:50
 * TODO:
 *
*/
#include "bwlabel.h"
#include "igl/unique.h"
#include "halfedge.h"
bool qrcode::bwlabel(Eigen::MatrixXd & BW, int connectivity, Eigen::MatrixXd & L)
{
	Engine* engine;
	igl::matlab::mlinit(&engine);
	igl::matlab::mlsetmatrix(&engine, "BW", BW);
	if (connectivity == 8)
		igl::matlab::mleval(&engine, "L=bwlabel(BW,8);");
	else if (connectivity == 4)
		igl::matlab::mleval(&engine, "L=bwlabel(BW,4);");
	else
		return false;
	igl::matlab::mlgetmatrix(&engine, "L", L);
	return true;
}

bool qrcode::bwindex(Eigen::MatrixXd & L, std::vector<Eigen::Matrix3d>& V_index)
{
	Eigen::MatrixXd C;
	std::vector<std::vector<int>> I;	igl::unique(L, C);
	int index = C.rows() - 1;
	I.resize(index);
	V_index.resize(index);
	for (int i = 0; i < L.rows(); i++) {
		for (int j = 0; j < L.cols(); j++) {
			if(round(L(i,j))!=0)
				I[round(L(i, j))-1].push_back(i*L.cols() + j);
		}
	}
	for (int i = 0; i < index; i++) {
		qrcode::eList *elist = new qrcode::eList();
		Eigen::MatrixXi E;
		for (int j = 0; j < I[i].size(); j++) {
			int a = I[i][j];
			int b = I[i][j] + L.cols();
			int c = I[i][j] + 1;
			int d = I[i][j] + L.cols() + 1;
			
			elist->add(a, b, 0);
			elist->add(b, d, 0);
			elist->add(d, c, 0);
			elist->add(c, a, 0);
		}
		elist->matrix(E);
		for (int i = 0; i < E.rows(); i++) {
			int x1 = E(i, 0) / L.cols();
			int yi = E(i, 0) % L.cols();
		}
	}
	return true;
}
