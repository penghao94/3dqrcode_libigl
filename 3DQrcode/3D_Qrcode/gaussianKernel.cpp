/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * GaussianKern.cpp  2017/03/30 9:43
 * TODO:
 *
*/
#include "gaussianKernel.h"
const double PI = 3.1415926;
bool qrcode::gaussianKernel(int size, Eigen::MatrixXd & G)
{
	G.resize(size, size);
	double u = double(size) / 2;
	for (int i = 1; i <= size; i++) {
		double x = i - 0.5 - u;
		double temp = 0.5 / PI*exp(-x*x / 2);
		for (int j = 1; j <= size; j++) {
			double y = j - 0.5 - u;
			G(i - 1, j - 1) = temp*0.5 / PI*exp(-y*y / 2);
		}
	}
	G = G / G.sum();
	return true;
}
