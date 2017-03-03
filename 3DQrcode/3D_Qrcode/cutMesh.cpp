#include "cutMesh.h"
#include "halfedge.h"
#include <igl/unique.h>
bool qrcode::cutMesh(Eigen::MatrixXd & V, Eigen::MatrixXi & F, Eigen::MatrixXi & fid, Eigen::MatrixXd & _V, Eigen::MatrixXi & _F)
{
	using namespace std;
	Eigen::VectorXi F_i, V_e, V_r;
	Eigen::MatrixXi rest_F, Ee,temp;
	igl::unique(fid, F_i);
	F_i.conservativeResize(F_i.rows() + 1);
	F_i(F_i.rows() - 1) = -1;
	_F.resize(F.rows() - F_i.rows() + 1, 3);
	rest_F.resize(F_i.rows() - 1, 3);
	int index = 0;
	int index_v = 0;
	for (int i = 0; i < F.rows(); i++) {
		if (i == F_i(index)) {
			rest_F.row(index) << F.row(i);
			index++;
		}
		else
			_F.row(i - index) << F.row(i);
	}

	qrcode::eList* elist = new qrcode::eList();

	for (int i = 0; i < rest_F.rows(); i++) {
		int id = F_i(i);
		elist->add(rest_F(i, 0), rest_F(i, 1), id);
		elist->add(rest_F(i, 1), rest_F(i, 2), id);
		elist->add(rest_F(i, 2), rest_F(i, 0), id);
	}
	elist->matrix(Ee);


	igl::unique(rest_F, V_r);
	temp = Ee.block(0, 0, Ee.rows(), 2);
	igl::unique(temp, V_e);
	_V = V;
	V_e.conservativeResize(V_e.rows() + 1);
	V_e(V_e.rows() - 1) = -1;
	for (int i = 0; i < V_r.rows(); i++) {
		if (V_r(i) == V_e(index_v)) {
			if (index_v < V_e.rows() - 1)
				index_v++;
		}
		else
			_V.row(V_r(i)) << _V.row(0);
	}
	return true;
}

bool qrcode::cutMesh(Eigen::MatrixXd & V, Eigen::MatrixXi & F, Eigen::MatrixXi & fid, Eigen::MatrixXd & _V, Eigen::MatrixXi & _F,Eigen::MatrixXi &Ee)
{
	using namespace std;
	Eigen::VectorXi F_i, V_e, V_r;
	Eigen::MatrixXi rest_F,temp;
	igl::unique(fid, F_i);
	F_i.conservativeResize(F_i.rows() + 1);
	F_i(F_i.rows() - 1) = -1;
	_F.resize(F.rows() - F_i.rows() + 1, 3);
	rest_F.resize(F_i.rows() - 1, 3);
	int index = 0;
	int index_v = 0;
	for (int i = 0; i < F.rows(); i++) {
		if (i == F_i(index)) {
			rest_F.row(index) << F.row(i);
			index++;
		}
		else
			_F.row(i - index) << F.row(i);
	}

	qrcode::eList* elist = new qrcode::eList();

	for (int i = 0; i < rest_F.rows(); i++) {
		int id = F_i(i);
		elist->add(rest_F(i, 0), rest_F(i, 1), id);
		elist->add(rest_F(i, 1), rest_F(i, 2), id);
		elist->add(rest_F(i, 2), rest_F(i, 0), id);
	}
	elist->matrix(Ee);


	igl::unique(rest_F, V_r);
	temp = Ee.block(0, 0, Ee.rows(), 2);
	igl::unique(temp, V_e);
	_V = V;
	V_e.conservativeResize(V_e.rows() + 1);
	V_e(V_e.rows() - 1) = -1;
	for (int i = 0; i < V_r.rows(); i++) {
		if (V_r(i) == V_e(index_v)) {
			if (index_v < V_e.rows() - 1)
				index_v++;
		}
		else
			_V.row(V_r(i)) << _V.row(0);
	}
	return true; 
}
