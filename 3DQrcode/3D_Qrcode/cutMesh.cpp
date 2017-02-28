#include "cutMesh.h"
#include "halfedge.h"
#include "igl/unique.h"
bool qrcode::cutMesh(Eigen::MatrixXd & V, Eigen::MatrixXi & F, Eigen::MatrixXi & fid, Eigen::MatrixXd & _V, Eigen::MatrixXi & _F)
{
	using namespace std;
	Eigen::VectorXi F_i, V_i;
	Eigen::MatrixXi rest_F,Ee;
	igl::unique(fid, F_i);
	F_i.conservativeResize(F_i.rows() + 1);
	F_i(F_i.rows() - 1) = -1;
	_F.resize(F.rows() - F_i.rows()+1, 3);
	rest_F.resize(F_i.rows()-1, 3);
	int index = 0;
	for (int i = 0; i < F.rows(); i++) {
		if (i == F_i(index)) {
			rest_F.row(index) << F.row(i);
			index++;
		}
		else
			_F.row(i - index) << F.row(i);
	}
	//cout << "rest_F:" << endl << rest_F<<endl;
	//cout << "_F:" << endl << _F<<endl;
	
	qrcode::eList* elist = new qrcode::eList();

	for (int i = 2; i < rest_F.rows(); i++) {
		elist->add(rest_F(i, 0), rest_F(i, 1));
		elist->add(rest_F(i, 1), rest_F(i, 2));
		elist->add(rest_F(i, 2), rest_F(i, 0));
	}
	//cout << "rest_F(0, 0)" << rest_F(0, 0) << "rest_F(0, 1)" << rest_F(0, 1);
	/*elist->add(rest_F(0, 0), rest_F(0, 1));
	elist->add(rest_F(0, 1), rest_F(0, 2));
	elist->add(rest_F(0, 2), rest_F(0, 0));
	*/
	elist->matrix(Ee);
	std::cout << "Ee:" << endl << Ee << std::endl;
	return true;
}

bool qrcode::cutMesh(Eigen::MatrixXd & V, Eigen::MatrixXi & F, Eigen::MatrixXi & fid, Eigen::MatrixXd & _V, Eigen::MatrixXi & _F, Eigen::MatrixXd Ev, Eigen::MatrixXi Ef, Eigen::MatrixXi Ee)
{
	/*Eigen::VectorXi F_i,V_i;
	Eigen::MatrixXi rest_F;
	igl::unique(fid, F_i);
	_F.resize(F.rows() - F_i.size(),3);
	rest_F.resize(F_i.size(),3);
	int index = 0;
	for (int i = 0; i < F.rows(); i++) {
		if (i == F_i(index)) {
			rest_F.row(index) << F.row(i);
			index++;
		}
			
		else
			_F.row(i - index) << F(i);
	}
	qrcode::eList* elist = new qrcode::eList();
	for (int i = 0; i < rest_F.rows(); i++) {
		elist->add(rest_F(i, 0), rest_F(i, 1));
		elist->add(rest_F(i, 1), rest_F(i, 2));
		elist->add(rest_F(i, 2), rest_F(i, 0));
	}
	elist->matrix(Ee);*/

	return true;
}
