#include "img_to_mesh.h"
#include "igl/unproject_onto_mesh.h"
#include <iostream>

bool qrcode::img_to_mesh(igl::viewer::Viewer & viewer, 
	Eigen::MatrixXd & V, 
	Eigen::MatrixXi & F, 
	Eigen::MatrixXi & D, 
	Eigen::MatrixXi & fid, 
	Eigen::MatrixXd & _V, 
	Eigen::MatrixXi & _F,
	Eigen::MatrixXd & _C)
{
	Eigen::Vector3f _uv;
	Eigen::Vector3d v0,v1,v2,_v;
	_V.resize(D.rows()*D.cols(),3);
	fid.resize(D.rows(),D.cols());
	double CENT_X = viewer.core.viewport(2)/2;
	double CENT_Y = viewer.core.viewport(3)/2;
	for (int i = 0; i < D.rows(); i++) {
		for (int j = 0; j < D.cols(); j++) {
			double x = i + CENT_X - D.rows() / 2;
			double y = CENT_Y + D.cols() / 2 - j;

			if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core.view*viewer.core.model,
				viewer.core.proj, viewer.core.viewport, V, F, fid(i, j), _uv)) {
				//std::cout<<fid(i,j)<<std::endl << temp_V << std::endl;
				v0 = V.row(F(fid(i, j), 0));
				v1 = V.row(F(fid(i, j), 1));
				v2 = V.row(F(fid(i, j), 2));
				_v = _uv(0)*v0 + _uv(1)*v1 + _uv(2)*v2;
				_V.row(i*D.cols() + j) << _v(0),_v(1),_v(2);
			}
			else 
				_V.row(i*D.cols() + j) << 0, 0, 0;
		}
	}
	img_to_facet(D, _F, _C);
	return true;
}

bool qrcode::img_to_facet(Eigen::MatrixXi & D, Eigen::MatrixXi & F, Eigen::MatrixXd & C)
{
	F.resize(2 * (D.rows() - 1)*(D.cols() - 1), 3);
	C.resize(2 * (D.rows() - 1)*(D.cols() - 1), 3);
	for (int i = 0; i < D.rows()-1; i++) {
		for (int j = 0; j < D.cols()-1; j++) {
			int a = i*D.cols() + j;
			int b = (i + 1)*D.cols() + j;
			int c = i*D.cols() + j + 1;
			int d = (i + 1)*D.cols() + j + 1;
			F.row(2 * (i*(D.cols() - 1) + j)) << a, b, c;
			F.row(2 * (i*(D.cols() - 1) + j) +1) << b, d, c;
			if (D(i,j) + D(i+1,j) +D(i,j+1) < 1)
				C.row(2*(i*(D.cols() - 1) + j)) << 0, 0, 0;
			else
				C.row(2 * (i*(D.cols() - 1) + j)) << 1.0, 1.0, 1.0;

			if (D(i+1, j+1) + D(i + 1, j) + D(i, j + 1)< 1)
				C.row(2 * (i*(D.cols() - 1) + j) +1) << 0, 0, 0;
			else
				C.row(2 * (i*(D.cols() - 1) + j) +1) << 1.0, 1.0, 1.0;
		}
	}
	return true;
}
