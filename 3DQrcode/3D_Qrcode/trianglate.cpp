#include "trianglate.h"
#include <igl/triangle/triangulate.h>
#include<igl/unique.h>
#include<iostream>
bool qrcode::tranglate(Eigen::MatrixXd & V1, Eigen::MatrixXi & E1, Eigen::MatrixXd & V2, Eigen::MatrixXi & E2, Eigen::MatrixXd & H, Eigen::MatrixXd &_V, Eigen::MatrixXi &_F)
{
	using namespace std;
	Eigen::MatrixXd V,_H,_V1,_V2;
	Eigen::MatrixXi C1,C2,IA1, IA2, IC1, IC2,E,_E1,_E2;
	igl::unique(E1, C1, IA1, IC1);
	igl::unique(E2, C2, IA2, IC2);
	V.resize(C1.rows() + C2.rows(),2);
	E.resize(E1.rows() + E2.rows(),2);
	_V1.resize(C1.rows(), 2);
	_V2.resize(C2.rows(), 2);
	_E1.resize(E1.rows(), 2);
	_E2.resize(E2.rows(), 2);
	for (int i = 0;i < C1.rows(); i++) {
		_V1.row(i) << V1(C1(i), 0), V1(C1(i), 1);
	}
	for (int i = 0; i < E1.rows(); i++) {
		_E1.row(i) << IC1(i), IC1(E1.rows()+i);
	}
	for (int i = 0; i < C2.rows(); i++) {
		_V2.row(i) << V2(C2(i), 0), V2(C2(i), 1);
	}
	for (int i = 0; i < E2.rows(); i++) {
		_E2.row(i) << IC2(i),IC2(E2.rows() + i);
	}
	cout << _V1 << endl << endl <<
		_E1 << endl << endl <<
		_V2 << endl << endl << _E2 << endl;
	V.block(0, 0, _V1.rows(), 2) << _V1;
	V.block(_V1.rows(), 0, _V2.rows(), 2) << _V2;
	E.block(0, 0, _E1.rows(), 2) << _E1;
	E.block(_E1.rows(), 0, _E2.rows(), 2) << (_E2.array()+_E1.rows()).matrix();
	igl::triangle::triangulate(V,E,H,"0.5",_V,_F);
	/**/
	return false;
}
