#include "trianglate.h"
#include <igl/triangle/triangulate.h>
#include<igl/unique.h>
#include<iostream>
bool qrcode::tranglate(Eigen::MatrixXd & V1, Eigen::MatrixXi & E1, Eigen::MatrixXd & V2, Eigen::MatrixXi & E2, Eigen::MatrixXd & H, Eigen::MatrixXd &_V, Eigen::MatrixXi &_F)
{
	using namespace std;
	Eigen::MatrixXd V,_H;
	Eigen::MatrixXi C1,C2,IA1, IA2, IC1, IC2,E;
	igl::unique(E1, C1, IA1, IC1);
	igl::unique(E2, C2, IA2, IC2);
	V.resize(C1.rows() + C2.rows(),2);
	E.resize(E1.rows() + E2.rows(),2);
	cout << V.rows()<<"   "<<E.rows() << endl;
	for (int i = 0;i < C1.rows(); i++) {
		V.row(i) << V1(C1(i), 0), V1(C1(i), 1);
	}
	for (int i = 0; i < E1.rows(); i++) {
		E.row(i) << IC1(2 * i), IC1(2 * i + 1);
	}
	for (int i = 0; i < C2.rows(); i++) {
		//V.row(C1.rows() + i) << V2(C2(i), 0), V2(C2(i), 1);
	}
	for (int i = 0; i < E2.rows(); i++) {
		E.row(C1.rows() + i) << IC2(2 * i) + E1.rows() - 1, IC2(2 * i + 1) + E1.rows() - 1;
	}
	igl::triangle::triangulate(V,E,H,"a0.5q",_V,_F);
	/**/
	return false;
}
