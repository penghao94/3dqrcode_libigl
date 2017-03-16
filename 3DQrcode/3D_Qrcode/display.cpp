/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * display.cpp  2017/03/14 9:28
 * TODO:
 *
*/
#include "display.h"

bool qrcode::display(Eigen::MatrixXd & V1, Eigen::MatrixXi & F1, Eigen::MatrixXd & C1, Eigen::MatrixXi &F_hit, Eigen::MatrixXd & V2, Eigen::MatrixXi & F2, Eigen::MatrixXd & C2, Eigen::MatrixXd & V, Eigen::MatrixXi & F, Eigen::MatrixXd & C)
{
	Eigen::MatrixXd _V;
	_V = V2;
	_V.col(2) << _V.col(2) *1.01;
	V.resize(V1.rows() + V2.rows(),3);
	F.resize(F1.rows() + F2.rows(),3);
	C.resize(C1.rows() + C2.rows(),3);

	V.block(0, 0, V1.rows(), 3) << V1;
	V.block(V1.rows(), 0, V2.rows(), 3) << _V;

	F.block(0, 0, F1.rows(), 3) << F1;
	F.block(F1.rows(), 0, F2.rows(), 3) << (F2.array()+V1.rows()).matrix();

	C.block(0, 0, C1.rows(), 3) << C1;
	C.block(C1.rows(), 0, C2.rows(), 3) << C2;

	for (int i = 0; i < F_hit.rows(); i++) {
		for (int j = 0; j < F_hit.cols(); j++) {
			C.row(F_hit(i, j)) << 1.0, 0.0, 0.0;
		}
	}
	return true;
}
