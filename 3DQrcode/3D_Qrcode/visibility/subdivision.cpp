#include "subdivision.h"
#include "igl/barycenter.h"
#include <omp.h>
bool qrcode::subdivision(Eigen::MatrixXd & V, Eigen::MatrixXi & F, Eigen::Matrix4f &mode,Eigen::MatrixXd & Vs, std::vector<Eigen::MatrixXi> & Fs, std::vector<Eigen::MatrixXi> &S)
{
	using namespace std;
	Eigen::Matrix4f model;
	model = mode.inverse().eval();
	Eigen::MatrixXd _V;
	_V.setOnes(V.rows(), 4);
	_V.block(0,0,V.rows(),3)<< V;
	_V = (mode*(_V.transpose().cast<float>())).transpose().cast<double>().block(0, 0, _V.rows(), 3);
	Eigen::MatrixXd BC,point;
	Eigen::RowVectorXd min_point(3), max_point(3), dir(3),centroid(3);
	igl::barycenter(_V, F, BC);
	point.resize(BC.rows(), 3);
	min_point = _V.colwise().minCoeff();
	max_point = _V.colwise().maxCoeff();
	centroid = (0.5*(min_point + max_point)).eval();
	Vs.resize(11, 3);
	double n = 100;
	double d = (max_point - centroid).norm();
	Vs.row(8) << centroid;
	dir << 0, 0, 1;
	Vs.row(9) << centroid + n * d*dir;
	dir << 0, 0, -1;
	Vs.row(10) << centroid + n * d*dir;
	dir << 0, 0, 0;
	Vs.row(0) << centroid + n * d*dir;
	dir << sqrt(0.5), sqrt(0.5), 0;
	Vs.row(1) << centroid + n * d*dir;
	dir << 0, 1, 0;
	Vs.row(2) << centroid + n * d*dir;
	dir << -sqrt(0.5), sqrt(0.5), 0;
	Vs.row(3) << centroid + n * d*dir;
	dir << -1, 0, 0;
	Vs.row(4) << centroid + n * d*dir;
	dir << -sqrt(0.5), -sqrt(0.5), 0;
	Vs.row(5) << centroid + n * d*dir;
	dir << 0, -1, 0;
	Vs.row(6) << centroid + n * d*dir;
	dir << sqrt(0.5), -sqrt(0.5), 0;
	Vs.row(7) << centroid + n * d*dir;
	Vs.conservativeResize(11, 4);
	Vs.col(3).setConstant(1);
	Vs= (model*(Vs.transpose().cast<float>())).transpose().cast<double>().block(0, 0, Vs.rows(), 3);

	Fs.resize(16);
	for (int i = 0; i < 8; i++) {
		Fs[i].resize(4, 3);
		Fs[i] << 8, i % 8, 9,
			8, 9, (i + 1) % 8,
			i % 8, 8, (i + 1) % 8,
			i % 8, (i + 1) % 8, 9;
		Fs[i+8].resize(4, 3);
		Fs[i+8] << 10, i % 8, 8,
			10, 8, (i + 1) % 8,
			(i + 1) % 8, 8, i % 8,
			(i + 1) % 8, i % 8, 10;
	}
	std::vector<std::vector<Eigen::VectorXi>>T;
	T.resize(16);
	S.resize(16);
	point = BC - centroid.replicate(BC.rows(),1);
	#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < BC.rows(); i++) {
		double a, b, c;
		if (point(i,2) >= 0)
			a = 0;
		else
			a = 1;
		if (point(i, 1) >= 0)
			b = 0;
		else
			b = 1;
		if (point(i, 0) >= 0) {
			if (tan(point(i, 1) / (point(i, 0)+0.0000001)) <= 1)
				c = 0;
			else
				c = 1;
		}
		else {
			if (tan(point(i, 1) / point(i, 0)) <= -1)
				c = 2;
			else
				c = 3;
		}
		T[a * 8 + b * 4 + c] .push_back (F.row(i).transpose());
	}
	#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < 16; i++) {
		S[i].resize(T[i].size(), 3);
		for (int j=0;j<T[i].size();j++)
		{
			S[i].row(j) << T[i][j].transpose();
		}
		T[i].clear();
	 }
	T.clear();
	BC.resize(0, 0);
	point.resize(0, 0);
	min_point.resize(0);
	max_point.resize(0);
	dir.resize(0);
	return false;
}
