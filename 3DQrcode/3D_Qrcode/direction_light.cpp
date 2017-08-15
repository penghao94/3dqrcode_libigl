#include "direction_light.h"
#include "curve_down.h"
#include "trianglate.h"
#include "bwlabel.h"
#include "gaussianKernel.h"
#include "mesh.h"
#include "optimization.h"
#include "visible_mesh.h"
#include "pre_pixel_normal.h"
#include "illuminate_map.h"
#include "ambient_occlusion.h"
#include <igl/Timer.h>
#include <igl/parallel_for.h>
#include <igl/writeOBJ.h>
#include <igl/jet.h>
#include <igl/barycenter.h>
#include <igl/embree/EmbreeIntersector.h>
#include <igl/matlab/MatlabWorkspace.h>
#include <Eigen/dense>
#include <Eigen/core>
#include <igl/writeOBJ.h>
#include <string.h>

bool qrcode::direction_light(igl::viewer::Viewer & viewer, Engine * engine, Eigen::Vector4d & position, Eigen::MatrixXd & D, Eigen::Matrix4f & mode,float zoom, int wht_num, int mul, int ext, Eigen::MatrixXd & V_uncrv, Eigen::MatrixXi & F_qr, Eigen::MatrixXd & C_qr, Eigen::MatrixXi & E_qr, Eigen::MatrixXd & H_qr, Eigen::MatrixXf & Src, Eigen::MatrixXf & Dir, std::vector<Eigen::MatrixXd>& th, Eigen::MatrixXd & V_pxl, Eigen::MatrixXd & V_rest, Eigen::MatrixXi & F_rest, Eigen::MatrixXi & E_rest, Eigen::MatrixXd & V_fin, Eigen::MatrixXi & F_fin)
{
	using namespace std;
	using namespace Eigen;

	igl::Timer timer;

	MatrixXd th_crv;//depth Matrix

	th_crv.setZero(D.rows(),D.cols());

	MatrixXd V_qr;	//Qr code vertices

	/*primary carve depth*/

	double step=1000;//step of iterator

	for (int i = 0; i < th[0].rows() - 1; i++) {
		for (int j = 0; j < th[0].cols() - 1; j++) {
			double temp = (V_pxl.row((i + 1)*th[0].cols() + j + 1) - V_pxl.row(i*th[0].cols() + j)).norm()/abs(Dir(i+D.cols()+j,2));
			if (temp < step) step = temp;
		}
	}

	/*Qr code mesh generation*/

	MatrixXi F_tri;	//triangulate mesh
	
	

	timer.start();

	qrcode::curve_down(V_uncrv, D, Src, Dir, th[0], wht_num, mul, th_crv, V_qr);// Carve down

	qrcode::tranglate(V_rest, E_rest, V_qr, E_qr, H_qr, mode, F_tri);//triangulate

	V_fin.resize(V_rest.rows() + V_qr.rows(), 3);
	F_fin.resize(F_rest.rows() + F_qr.rows() + F_tri.rows(), 3);
	V_fin.block(0, 0, V_rest.rows(), 3) << V_rest;
	V_fin.block(V_rest.rows(), 0, V_qr.rows(), 3) << V_qr;
	F_fin.block(0, 0, F_rest.rows(), 3) << F_rest;
	F_fin.block(F_rest.rows(), 0, F_qr.rows(), 3) << (F_qr.array() + V_rest.rows()).matrix();
	F_fin.block(F_rest.rows() + F_qr.rows(), 0, F_tri.rows(), 3) << F_tri;

	cout << "Remesh time = " << timer.getElapsedTime() << endl;

	/*Qr code simulation*/

	timer.start();

	MatrixXd BW, _BW;	//BW label

	qrcode::bwlabel(engine, D, 4, BW);
	cout << BW << endl;
	_BW.setZero((BW.rows() - 1) / ext + 1, (BW.rows() - 1) / ext + 1);

	for (int i = 0; i < _BW.rows() - 1; i++) {
		for (int j = 0; j < _BW.cols() - 1; j++) {
			_BW(i, j) = BW(ext*i, ext*j);
		}
	}
	
	MatrixXd K;	//Gaussian kernel
	
	int scale = 1;	//scale of data D

	qrcode::gaussianKernel(scale, K);	//Gaussian kernel

	vector<MatrixXi> B_cxx; //2D visible boundary

	qrcode::bwindex(_BW, ext, B_cxx);//BW label to 2D boundary

	std::vector<qrcode::Mesh> meshes = qrcode::visible_mesh(BW, B_cxx, V_pxl); //2D boundary to 3D boundary

	igl::matlab::mleval(&engine, "clc,clear");
	igl::matlab::mlsetmatrix(&engine, "W", BW);
	igl::matlab::mleval(&engine, "[a,b]=size(W)");
	igl::matlab::mleval(&engine, "W=W(1:(a-1),1:(b-1))");
	igl::matlab::mleval(&engine, "w=length(find(W==0))");

	int white = igl::matlab::mlgetscalar(&engine, "w");	//number of  white pixel
	int black = pow(BW.cols() - 1, 2) - white;	//number of black pixel

	MatrixXd Pb, Nb, Pw, Nw;	//pre_calculate point and normal of black and white pixel

	qrcode::pre_black_normal(BW, Src, Dir, th[0], th_crv, scale, black, Pb, Nb);

	qrcode::pre_white_normal(BW, V_pxl, scale, white, Pw, Nw);

	Matrix4f model;

	model = mode.inverse().eval();

	MatrixXd _V;

	_V.setOnes(V_fin.rows(), 4);
	_V.block(0, 0, V_fin.rows(), 3) << V_fin;
	_V = (mode*(_V.transpose().cast<float>())).transpose().cast<double>().block(0, 0, _V.rows(), 3);

	MatrixXd BC;

	igl::barycenter(_V, F_fin, BC);
	
	VectorXd centr(3);	//centroid of model

	centr = 0.5*(BC.colwise().maxCoeff() + BC.colwise().minCoeff()).eval();

	VectorXd dir(3), pos(3);// user design position

	dir << position(0), position(1), position(2);

	pos = centr + position(3)*zoom*dir;

	centr.conservativeResize(4);

	centr.col(3).setConstant(1);

	centr = (model*(centr.cast<float>())).cast<double>().block(0, 0, 3,1);

	pos.conservativeResize(4);

	pos.col(3).setConstant(1);

	pos= (model*(pos.cast<float>())).cast<double>().block(0,0, 3,1);

	dir = (pos - centr).normalized();

	igl::embree::EmbreeIntersector ei;	//rays

	VectorXd Bmap, Wmap;// is_light result of black and white pixel;

	qrcode::is_light(ei, V_fin, F_fin, pos, Pw, Wmap);//1 is light ,0 is dark

	if (Wmap.minCoeff()==0.0) {
		cout << "White block can't be lighted!!!" << endl;
		 return false;
	}
	qrcode::is_light(ei, V_fin, F_fin, pos, Pb, Bmap);

	MatrixXd Light(BW.rows() - 1, BW.cols() - 1);//direction light result

	const auto range = [&dir,&pos](RowVectorXd des )->bool {
		VectorXd d = (pos - des.transpose()).normalized();
		if (sqrt(1.0 - d.dot(dir)) > 1.0 / sqrt(101.0))
			return false;
		else
			return true;
	};

	int windex = 0, bindex = 0;
	/*Qr code optimization	the first iterator*/

	igl::matlab::MatlabWorkspace mw;

	bool stop = true; //ending condition

	int it = 0;	//index of iterator

	for (int i = 0; i < BW.rows() - 1; i++) {
		for (int j = 0; j < BW.cols() - 1; j++) {

			if (BW(i, j) > 0) {
				for (int m = 0; m < scale; m++) {
					for (int n = 0; n < scale; n++) {

						/*Determine if Qr code is in the range of direction light*/
						if (!range(V_pxl.row((i*scale + m)*th[0].cols() + j*scale + n))) {
							cout << "Qr code is out range of direction lighting!!!" << endl;
							return false;
						}

						Light(i*scale + m, j*scale + n) = Bmap(bindex);

						if (Bmap(bindex) != 0) {
							th_crv(i*scale + m, j*scale + n) += step;
							stop = false;
						}
						bindex++;
					}
				}

			}
			else {

				for (int m = 0; m < scale; m++) {
					for (int n = 0; n < scale; n++) {

						if (!range(V_pxl.row((i*scale + m)*th[0].cols() + j*scale + n))) {
							cout << "Qr code is out range of direction lighting!!!" << endl;
							return false;
						}

						Light(i*scale + m, j*scale + n) = Wmap(windex);
						windex++;
					}
				}

			}

		}
	}

	mw.save(Light, "L");

	mw.write("result/Experiment_" + to_string(it) + ".mat");

	qrcode::curve_down(V_uncrv, D, Src, Dir, th[0], wht_num, mul, th_crv, V_qr);// Carve down

	V_fin.block(V_rest.rows(), 0, V_qr.rows(), 3) << V_qr;

	qrcode::pre_black_normal(BW, Src, Dir, th[0], th_crv, scale, black, Pb, Nb);

	igl::writeOBJ("result/Experiment_" + to_string(it) + ".obj", V_fin, F_fin);

	/*more iterator*/

	while (!stop) {

		stop = true;

		it++;

		qrcode::is_light(ei, V_fin, F_fin, pos, Pb, Bmap);

		int index = 0;
		for (int i = 0; i < BW.rows() - 1; i++) {
			for (int j = 0; j < BW.cols() - 1; j++) {

				if (BW(i, j) > 0) {
					for (int m = 0; m < scale; m++) {
						for (int n = 0; n < scale; n++) {

							Light(i*scale + m, j*scale + n) = Bmap(index);

							if (Bmap(index) != 0) {

								th_crv(i*scale + m, j*scale + n) += step;
								stop = false;
							}
							index++;

						}
					}

				}

			}
		}

		mw.save(Light, "L");

		mw.write("result/Experiment_" + to_string(it) + ".mat");

		qrcode::curve_down(V_uncrv, D, Src, Dir, th[0], wht_num, mul, th_crv, V_qr);// Carve down

		V_fin.block(V_rest.rows(), 0, V_qr.rows(), 3) << V_qr;

		qrcode::pre_black_normal(BW, Src, Dir, th[0], th_crv, scale, black, Pb, Nb);

		igl::writeOBJ("result/Experiment_" + to_string(it) + ".obj", V_fin, F_fin);

	}

	/*Qr code simulation in ambient light condition*/

	VectorXd Sw, Sb; //sequence result of ambient occlusion

	VectorXd Ar;	//sequence result of spherical Area;

	qrcode::ambient_occlusion(V_fin, F_fin, Pw, Nw, 5000, Sw);

	qrcode::ambient_occlusion(V_fin, F_fin, Pb, Nb, meshes, Ar, Sb);

	MatrixXd AO;	//Ambient occlusion;

	MatrixXd AE;	//Spherical area

	AO.setZero((BW.rows() - 1)*scale, (BW.cols() - 1)*scale);

	AE.setZero((BW.rows() - 1)*scale, (BW.cols() - 1)*scale);

	bindex = 0; windex = 0;

	for (int i = 0; i < BW.rows() - 1; i++) {
		for (int j = 0; j < BW.cols() - 1; j++) {

			if (BW(i, j) == 0) {

				for (int m = 0; m < scale; m++) {
					for (int n = 0; n < scale; n++) {
						int x = i*scale + m;
						int y = j*scale + n;
						AO(x, y) = Sw(windex);
						windex++;
					}
				}

			}
			else {

				for (int m = 0; m < scale; m++) {
					for (int n = 0; n < scale; n++) {
						int x = i*scale + m;
						int y = j*scale + n;
						AO(x, y) = Sb(bindex);
						AE(x, y) = Ar(bindex);
						bindex++;
					}
				}

			}

		}
	}

	mw.save(AO, "AO");

	mw.save(AE, "AE");

	MatrixXi G;		//Gray value

	qrcode::vis2gray(engine, AO, G);

	mw.save(G, "G");

	mw.write("Ambient.mat");

	ei.global_deinit();

	viewer.data.clear();

	viewer.data.set_face_based(true);

	viewer.data.set_mesh(V_fin, F_fin);

	cout << "Optimized time = " << timer.getElapsedTime() << endl;
	return true;
}
