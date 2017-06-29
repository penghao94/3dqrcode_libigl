#include "optimization.h"
#include "curve_down.h"
#include "trianglate.h"
#include "cut_plane.h"
#include "bwlabel.h"
#include "visibility/visibility.h"
#include "visibility/gaussianKernel.h"
#include <igl/writeOBJ.h>
#include "2d_visible.h"
#include "mesh.h"
#include "visible_mesh.h"
#include "pre_pixel_normal.h"
#include "ambient_occlusion.h"


bool qrcode::vis2gray(Engine *engine,Eigen::MatrixXd & vis, Eigen::MatrixXi & gray)
{
	 gray.resize(vis.rows(),vis.cols());
	 igl::matlab::mleval(&engine, "clc,clear");
	igl::matlab::mlsetmatrix(&engine, "v", vis);
	igl::matlab::mleval(&engine, "g=round(195.6.*v.^0.09219)");
	igl::matlab::mlgetmatrix(&engine, "g", gray);

	return false;
}

bool qrcode::gray2vis(Engine *engine,Eigen::MatrixXi & gray, Eigen::MatrixXd & vis)
{
	igl::matlab::mleval(&engine, "clc,clear");
	igl::matlab::mlsetmatrix(&engine, "g", gray);
	igl::matlab::mleval(&engine, "v=5.814e-28.*exp(0.288.*g)");
	igl::matlab::mlgetmatrix(&engine, "v", vis);
	return false;
}
bool qrcode::optimization(Engine * engine, Eigen::MatrixXd & D, Eigen::Matrix4f & mode, int wht_num, int mul, int ext,Eigen::MatrixXd & V_uncrv, Eigen::MatrixXi & F_qr, Eigen::MatrixXd & C_qr,
	Eigen::MatrixXi & E_qr, Eigen::MatrixXd & H_qr, Eigen::MatrixXf & Src, Eigen::MatrixXf & Dir, std::vector<Eigen::MatrixXd>& th, Eigen::MatrixXd & V_pxl,
	Eigen::MatrixXd & V_rest, Eigen::MatrixXi & F_rest, Eigen::MatrixXi & E_rest, Eigen::MatrixXd & V_fin, Eigen::MatrixXi& F_fin)
{
	using namespace std;
	igl::matlab::MatlabWorkspace mw;
	igl::Timer timer;
	//limitation of th
	Eigen::MatrixXd th_lim = ((th[1] - th[0]).array() / (th[1] - th[0]).maxCoeff()).matrix();
	/*
	initiate condition
	*/
	Eigen::MatrixXd th_crv(D.rows(), D.cols());
	Eigen::MatrixXd AG(D.rows() - 1, D.cols() - 1);
	Eigen::MatrixXd V_qr;
	//primary value of carve down
	double step = 0;
	for (int i = 0; i < th[0].rows() - 1; i++) {
		for (int j = 0; j < th[0].cols() - 1; j++) {
			step += (V_pxl.row((i + 1)*th[0].cols() + j + 1) - V_pxl.row(i*th[0].cols() + j)).norm();
		}
	}
	step /= ((th[0].rows() - 1)*(th[0].cols() - 1));
	int a, b;
	double c = (th[1] - th[0]).maxCoeff(&a, &b);
	double d = step / abs(Dir(a*th[0].cols() + b, 2));
	th_crv.setConstant(d);
	cout << "step:" << d << endl;
	//Triangulate mesh
	Eigen::MatrixXi F_tri;
	//BW parameter
	Eigen::MatrixXd BW,_BW;
	std::vector<Eigen::MatrixXi> B_cxx;
	//Visibility and gray parameter
	Eigen::MatrixXd Vis,A;
	Eigen::MatrixXi G;
	//Carve down 
	timer.start();
	qrcode::curve_down(V_uncrv, D, Src, Dir, th[0], wht_num, mul, th_crv, V_qr);
	//triangulate 
	qrcode::tranglate(V_rest, E_rest, V_qr, E_qr, H_qr, mode, F_tri);
	//Remesh
	V_fin.resize(V_rest.rows() + V_qr.rows(), 3);
	F_fin.resize(F_rest.rows() + F_qr.rows() + F_tri.rows(), 3);
	V_fin.block(0, 0, V_rest.rows(), 3) << V_rest;
	V_fin.block(V_rest.rows(), 0, V_qr.rows(), 3) << V_qr;
	F_fin.block(0, 0, F_rest.rows(), 3) << F_rest;
	F_fin.block(F_rest.rows(), 0, F_qr.rows(), 3) << (F_qr.array() + V_rest.rows()).matrix();
	F_fin.block(F_rest.rows() + F_qr.rows(), 0, F_tri.rows(), 3) << F_tri;
	cout << "Remesh time = " << timer.getElapsedTime() << endl;
	timer.start();
	// BW operation
	qrcode::bwlabel(engine, D, 4, BW);
	int scale = th[0].cols() / BW.cols();

	Eigen::MatrixXd K;
	qrcode::gaussianKernel(scale, K);
	Eigen::MatrixXd module(scale, scale);
	Eigen::MatrixXd Pb, Nb, Pw, Nw;
	Eigen::VectorXd Sw, Sb,Ar;

	_BW.setZero((BW.rows() - 1) / ext + 1, (BW.rows() - 1) / ext + 1);
	for (int i = 0; i < _BW.rows() - 1; i++) {
		for (int j = 0; j < _BW.cols() - 1; j++) {
			_BW(i, j) = BW(ext*i, ext*j);
		}
	}
	qrcode::bwindex(_BW,ext, B_cxx);
	std::vector<qrcode::Mesh> meshes = qrcode::visible_mesh(BW, B_cxx, V_pxl);
	igl::matlab::mleval(&engine, "clc,clear");
	igl::matlab::mlsetmatrix(&engine, "W", BW);
	igl::matlab::mleval(&engine, "[a,b]=size(W)");
	igl::matlab::mleval(&engine, "W=W(1:(a-1),1:(b-1))");
	igl::matlab::mleval(&engine, "w=length(find(W==0))");
	int white = igl::matlab::mlgetscalar(&engine, "w");
	int black = pow(BW.cols() - 1, 2) - white;
	qrcode::pre_black_normal(BW, Src, Dir, th[0], th_crv, scale, black, Pb, Nb);
	qrcode::pre_white_normal(BW, V_pxl, scale, white, Pw, Nw);

	qrcode::ambient_occlusion(V_fin, F_fin, Pw, Nw, 50000, Sw);
	//cout << Sw << endl;
	qrcode::ambient_occlusion(V_fin, F_fin, Pb, Nb, meshes,Ar,Sb);
	//qrcode::ambient_occlusion(V_fin, F_fin, Pb, Nb, 500000, Sb);
	//cout << Sb << endl;
	Vis.setZero((BW.rows() - 1)*scale, (BW.cols() - 1)*scale);
	A.setOnes((BW.rows() - 1)*scale, (BW.cols() - 1)*scale);
	int indexw = 0;
	int indexb = 0;
	for (int i = 0; i < BW.rows() - 1; i++) {
		for (int j = 0; j < BW.cols() - 1; j++) {
			if (BW(i, j) == 0) {
				for (int m = 0; m < scale; m++) {
					for (int n = 0; n < scale; n++) {
						int x = i*scale + m;
						int y = j*scale + n;
						Vis(x, y) = Sw(indexw);
						
						indexw++;
					}
				}
			}
			else {
				for (int m = 0; m < scale; m++) {
					for (int n = 0; n < scale; n++) {
						int x = i*scale + m;
						int y = j*scale + n;
						Vis(x, y) = Sb(indexb);
						A(x, y) = Ar(indexb);
						indexb++;
					}
				}
			}
		}
	}
	qrcode::vis2gray(engine, Vis, G);
	mw.save(A, "A");
	mw.save(Vis, "V");
	mw.write("result/Experiment_0.mat");
	igl::writeOBJ("result/Experiment_0.obj", V_fin, F_fin);
	cout << "Optimization 0 time = " << timer.getElapsedTime() << endl;
	int index = 0;
	Eigen::MatrixXi W(1, white*scale*scale);
	for (int i = 0; i < BW.rows() - 1; i++) {
		for (int j = 0; j < BW.cols() - 1; j++) {
			if (BW(i, j) == 0) {
				for (int k = 0; k < scale; k++) {
					for (int l = 0; l < scale; l++) {
						W(0, index) = G(i*scale + k, j*scale + l);
						index++;
					}
				}
			}
		}
	}
	igl::matlab::mleval(&engine, "clc,clear");
	igl::matlab::mlsetmatrix(&engine, "W", W);
	igl::matlab::mlsetscalar(&engine, "num", round(wht_num*scale*scale / 10));
	igl::matlab::mleval(&engine, "W=sort(W,'descend')");
	igl::matlab::mleval(&engine, "m=mean(W(1,1:num))");
	double gray_mean = igl::matlab::mlgetscalar(&engine, "m");
	int threshold = round(gray_mean - 255 * 0.2);
	cout << threshold << endl;
	int count = 1;
	bool stop = false;
	while (!stop)
	{
		stop = true;
		for (int i = 0; i < BW.rows() - 1; i++) {
			for (int j = 0; j < BW.cols() - 1; j++) {
					module = G.block(i*scale, j*scale, scale, scale).cast<double>();
				AG(i, j) = (K.array()*module.array()).matrix().sum();
				if (BW(i, j) != 0) {
					if (AG(i, j)> threshold) {
						th_crv(i, j) = th_crv(i, j) + d;
						stop = false;
					}
				}
			}
		}
		if (stop == false) {
			//Carve down 
			timer.start();
			qrcode::curve_down(V_uncrv, D, Src, Dir, th[0], wht_num, mul, th_crv, V_qr);
			//Remesh
			V_fin.block(V_rest.rows(), 0, V_qr.rows(), 3) << V_qr;
			cout << "Remesh time = " << timer.getElapsedTime() << endl;
			timer.start();
			qrcode::pre_black_normal(BW, Src, Dir, th[0], th_crv, scale, black, Pb, Nb);
			qrcode::ambient_occlusion(V_fin, F_fin, Pb, Nb, meshes,Ar,Sb);
			//cout << Sb << endl;
			int indexw = 0;
			int indexb = 0;
			for (int i = 0; i < BW.rows() - 1; i++) {
				for (int j = 0; j < BW.cols() - 1; j++) {
					if (BW(i,j)>0)
					{
						for (int m = 0; m < scale; m++) {
							for (int n = 0; n < scale; n++) {
								int x = i*scale + m;
								int y = j*scale + n;
								Vis(x, y) = Sb(indexb);
								A(x, y) = Ar(indexb);
								indexb++;
							}
						}
					}	
				}
			}
			qrcode::vis2gray(engine, Vis, G);
			igl::writeOBJ("result/Experiment_" + to_string(count) + ".obj", V_fin, F_fin);
			cout << "Optimization" << count << " time= " << timer.getElapsedTime() << endl;

		}
		mw.save(A, "A");
		mw.save(AG, "AG");
		mw.save(Vis, "V");
		mw.write("result/Experiment_" + to_string(count) + ".mat");
		count++;
	}
	return true;
}
