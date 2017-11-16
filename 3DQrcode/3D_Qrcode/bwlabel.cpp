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
#include "outBound.h"
bool qrcode::bwlabel(Engine *engine, Eigen::MatrixXd & BW, int connectivity, Eigen::MatrixXd & L)
{

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

bool qrcode::bwindex(Engine *engine,Eigen::MatrixXd &V_pxl,Eigen::MatrixXd & L,int scale, std::vector<Eigen::MatrixXd>& V_index)
{
	using namespace std;
	Eigen::MatrixXd C;
	Eigen::MatrixXi E;
	Eigen::MatrixXd V;
	Eigen::MatrixXi _E,temp;
	std::vector<std::vector<int>> I;	
	igl::unique(L, C);
	int index = C.rows() - 1;
	int col = L.cols()*scale;
	C.resize(0, 0);
	I.resize(index);
	V_index.resize(index); 

	for (int i = 0; i < L.rows(); i++) {
		for (int j = 0; j < L.cols(); j++) {
			if(round(L(i,j))!=0)
				I[round(L(i, j))-1].push_back(i*L.cols() + j);
		}
	}
	/*for (int i = 0; i < I[24].size(); i++) {
		cout << int(I[24][i] / L.cols()) << "     " << int(I[24][i] % L.cols()) << endl;
	}*/
	for (int i = 0; i < index; i++) {
		qrcode::eList *elist = new qrcode::eList();
		
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
		_E.resize(E.rows(), 2);
		temp.resize(2*E.rows(), 2);
		delete elist;
		for (int k = 0; k < E.rows(); k++) {
			if (k == 0) {
				_E.row(k) << E(k, 0), E(k, 1);
			}
			else {
				for (int l = 0; l < E.rows(); l++) {
					if (E(l, 0) == _E(k-1,1)) {
						_E.row(k) << E(l, 0), E(l, 1);
						break;
					}
				}
			}
		}
		V.resize(E.rows()*scale, 3);
		for (int k = 0; k < E.rows(); k++) {
			
			int x1 = _E(k, 0) % L.cols();
			int y1 = _E(k, 0) / L.cols();
			int x2 = _E(k, 1) % L.cols();
			int y2 = _E(k, 1) / L.cols();
			for (int m = 0; m < scale; m++) {
				V.row(scale * k + m) << V_pxl.row((y1*scale+(y2-y1)*m)*col+x1*scale+(x2-x1)*m);
				
			}
		}
		V_index[i] = V;
	}
	V.resize(0, 0);
	E.resize(0, 0);
	return true;
}

bool qrcode::bwindex(Engine * engine, Eigen::MatrixXd & L, int scale, std::vector<Eigen::MatrixXi>& V_index)
{
	using namespace std;
	Eigen::MatrixXd C;
	Eigen::MatrixXi E, _E;
	Eigen::MatrixXi outer,hole,temp;
	
	std::vector<std::vector<int>> I;
	igl::unique(L, C);
	int index = C.rows() - 1;
	int col = L.cols()*scale;
	C.resize(0, 0);
	I.resize(index);
	V_index.resize(index);

	for (int i = 0; i < L.rows(); i++) {
		for (int j = 0; j < L.cols(); j++) {
			if (round(L(i, j)) != 0) {
				I[round(L(i, j)) - 1].push_back(i*L.cols() + j);
			}
				
			
		}
	}
	for (int i = 0; i < index; i++) {
		qrcode::eList *elist = new qrcode::eList();
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
		delete elist;
		_E = E.block(0, 0, E.rows(), 2);
		hole = _E;
		/*temp.resize( outer.rows(), 2);
		for (int k = 0; k < outer.rows(); k++) {
			temp.row(k) << int(outer(k, 1) / L.cols()*scale), int(outer(k, 1) % L.cols()*scale);
		}	
		igl::matlab::mlsetmatrix(&engine, "temp",temp);
		igl::matlab::mlsetscalar(&engine, "i", i);
		igl::matlab::mleval(&engine, "figure(i)");
		igl::matlab::mleval(&engine, "plot(temp(:,2),-temp(:,1));");*/
		temp.resize(0, 0);
		while (hole.rows()!=0)
		{
			qrcode::outBound(hole, 0, L.cols(), outer);
			outer.cast<double>();
			igl::matlab::mlsetmatrix(&engine, "E",hole);
			igl::matlab::mlsetmatrix(&engine, "B", outer);
			igl::matlab::mleval(&engine, "H=setdiff(E,B,'rows');");
			igl::matlab::mlgetmatrix(&engine, "H", hole);
			outer.cast<int>();
			hole.cast<int>();
			temp.conservativeResize(temp.rows() + outer.rows(), 2);
			temp.block(temp.rows()-outer.rows(),0, outer.rows(),2)<<outer;
		}
		V_index[i] = temp;
	}
	
	E.resize(0, 0);
	_E.resize(0, 0);
	outer.resize(0, 0);
	hole.resize(0, 0);
	return true;
}

bool qrcode::bwindex(Eigen::MatrixXd & L, std::vector<Eigen::MatrixXi>& V_index)
{
	using namespace std;
	Eigen::MatrixXd C;
	Eigen::MatrixXi E, _E;

	std::vector<std::vector<int>> I;
	igl::unique(L, C);
	int index = C.rows() - 1;
	C.resize(0, 0);
	I.resize(index);
	V_index.resize(index);

	for (int i = 0; i < L.rows(); i++) {
		for (int j = 0; j < L.cols(); j++) {
			if (round(L(i, j)) != 0) {
				I[round(L(i, j)) - 1].push_back(i*L.cols() + j);
			}


		}
	}
	for (int i = 0; i < index; i++) {
		qrcode::eList *elist = new qrcode::eList();
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
		delete elist;
		_E = E.block(0, 0, E.rows(), 2);
		V_index[i] = _E;
	}
	E.resize(0, 0);
	_E.resize(0, 0);
	return true;
}
bool qrcode::bwindex(Eigen::MatrixXd & L, int scale, std::vector<Eigen::MatrixXi>& V_index)
{
	using namespace std;
	Eigen::MatrixXd C;
	Eigen::MatrixXi E, _E;
	int col = (L.cols() - 1)*scale + 1;
	std::vector<std::vector<int>> I;
	igl::unique(L, C);
	int index = C.rows() - 1;
	C.resize(0, 0);
	I.resize(index);
	V_index.resize(index);

	for (int i = 0; i < L.rows(); i++) {
		for (int j = 0; j < L.cols(); j++) {
			if (round(L(i, j)) != 0) {
				I[round(L(i, j)) - 1].push_back(i*scale*col + j*scale);
			}
		}
	}
	for (int i = 0; i < index; i++) {
		qrcode::eList *elist = new qrcode::eList();
		for (int j = 0; j < I[i].size(); j++) {
			int a = I[i][j];
			int b = I[i][j] + scale*col;
			int c = I[i][j] + scale;
			int d = I[i][j] + scale*col + scale;

			elist->add(a, b, 0);
			elist->add(b, d, 0);
			elist->add(d, c, 0);
			elist->add(c, a, 0);
		}
		elist->matrix(E);
		delete elist;
		_E = E.block(0, 0, E.rows(), 2);
		V_index[i] = _E;
	}
	E.resize(0, 0);
	_E.resize(0, 0);
	return true;
}
bool qrcode::upperpoint(Eigen::MatrixXd & V, Eigen::MatrixXi & F, Eigen::Matrix4f & mode,int rest_V, int wht_V, int rest_F, int wht_F, Eigen::VectorXi upnt, Eigen::VectorXi & ufct,int &v_num,int &f_num)
{
	v_num = 0;
	f_num = 0;
	int f = 0;
	Eigen::MatrixXd _V(V.rows(), 4);
	_V.block(0, 0, _V.rows(), 3) << V;
	_V.col(3).setConstant(1);
	_V = (mode*(_V.transpose().cast<float>())).transpose().cast<double>().block(0, 0, _V.rows(), 3);
	double minP = _V.block(rest_V, 2, wht_V, 1).minCoeff();

	upnt.setZero(V.rows());
	ufct.setZero(F.rows());

	for (int i = 0; i < _V.rows(); i++) {
		if (_V(i, 2) >= minP) {
			upnt(i) = 1;
			v_num++;
		}
	}

	for (int j = 0; j < F.rows(); j++) {
		if (upnt(F(j, 0)) == 1 || upnt(F(j, 1)) == 1 || upnt(F(j, 2)) == 1) {
			ufct(j) = 1;
			f_num++;
		}
	}


	return true;
}
