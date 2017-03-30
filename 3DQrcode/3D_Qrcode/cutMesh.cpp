#include "cutMesh.h"
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
	elist->matrix(temp);


	igl::unique(rest_F, V_r);
	Ee = temp.block(0, 0, temp.rows(), 2);
	igl::unique(Ee, V_e);
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
	Eigen::VectorXi F_i, V_e, V_r, V_i, V_s;
	Eigen::MatrixXi rest_F,temp;

	int index = 0;
	int index_v = 0;
	int index_i = 0;

	igl::unique(fid, F_i);
	F_i.conservativeResize(F_i.rows() + 1);
	F_i(F_i.rows() - 1) = -1;
	_F.resize(F.rows() - F_i.rows() + 1, 3);
	rest_F.resize(F_i.rows() - 1, 3);
	V_s.resize(V.rows());
	V_s.setConstant(-1);
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
	elist->matrix(temp);
	delete elist;
	igl::unique(rest_F, V_r);
	Ee = temp.block(0, 0, temp.rows(), 2);
	igl::unique(Ee, V_e);
	V_e.conservativeResize(V_e.rows() + 1);
	V_e(V_e.rows() - 1) = -1;
	V_i.resize(V_r.rows() - V_e.rows()+2);
	_V.resize(V.rows() - V_i.rows()+1,3);

	for (int i = 0; i < V_r.rows(); i++) {
		if (V_r(i) == V_e(index_v)) {
			if (index_v < V_e.rows() - 1)
				index_v++;
		}
		else {
			V_i(index_i) = V_r(i);
			index_i++;
		}
	}

	index_i = 0;
	index_v = 0;
	for (int i = 0; i < V.rows(); i++) {
		if (i != V_i(index_i)) {
			_V.row(index_v) << V.row(i);
			V_s(i) = index_v;
			index_v++;
		}
		else
			index_i++;
	}
	for (int i = 0; i < _F.rows(); i++) {
		for (int j = 0; j < 3; j++) {
			_F(i, j) = V_s(_F(i, j));
		}
	}
	for (int i = 0; i < Ee.rows(); i++) {
		for (int j = 0; j < 2; j++) {
			Ee(i, j) = V_s(Ee(i, j));
		}
	}
	return true; 
}
