#include "watermark.h"
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
#include <igl/embree/EmbreeIntersector.h>
#include <igl/matlab/MatlabWorkspace.h>
#include <Eigen/dense>
#include <igl/writeOBJ.h>
#include <string.h>
void qrcode::watermark(igl::viewer::Viewer &viewer, Engine * engine, Eigen::MatrixXd & D, Eigen::Matrix4f & mode, int wht_num, int mul, int ext,
	Eigen::MatrixXd & V_uncrv, Eigen::MatrixXi & F_qr, Eigen::MatrixXd & C_qr, Eigen::MatrixXi & E_qr,
	Eigen::MatrixXd & H_qr, Eigen::MatrixXf & Src, Eigen::MatrixXf & Dir, std::vector<Eigen::MatrixXd>& th, 
	Eigen::MatrixXd & V_pxl, Eigen::MatrixXd & V_rest, Eigen::MatrixXi & F_rest, Eigen::MatrixXi & E_rest, 
	Eigen::MatrixXd & V_fin, Eigen::MatrixXi & F_fin)
{
	using namespace std;
	using namespace Eigen;
	igl::Timer timer;
	//Initiate  variant
	MatrixXd th_crv(D.rows(), D.cols());
	MatrixXd V_qr;
	//primary carve depth
	double step = 0;
	for (int i = 0; i < th[0].rows() - 1; i++) {
		for (int j = 0; j < th[0].cols() - 1; j++) {
			step += (V_pxl.row((i + 1)*th[0].cols() + j + 1) - V_pxl.row(i*th[0].cols() + j)).norm();
		}
	}
	step /= ((th[0].rows() - 1)*(th[0].cols() - 1));
	int a = round((D.rows() - 1) / 2);
	int b = round((D.cols() - 1) / 2);
	double c = step / abs(Dir(a*D.cols() + b, 2))/100;
	th_crv.setConstant(c);
	//Triangulate mesh
	Eigen::MatrixXi F_tri;
	//BW parameter
	Eigen::MatrixXd BW, _BW;
	std::vector<Eigen::MatrixXi> B_cxx;

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
	//BW operation
	qrcode::bwlabel(engine, D, 4, BW);
	int scale = 1;

	//Gaussian Kernel
	Eigen::MatrixXd K;
	qrcode::gaussianKernel(scale, K);
	Eigen::MatrixXd module(scale, scale);
	//point and normal
	Eigen::MatrixXd Pb, Nb, Pw, Nw;
	Eigen::VectorXd Sw, Sb;
	_BW.setZero((BW.rows() - 1) / ext + 1, (BW.rows() - 1) / ext + 1);
	for (int i = 0; i < _BW.rows() - 1; i++) {
		for (int j = 0; j < _BW.cols() - 1; j++) {
			_BW(i, j) = BW(ext*i, ext*j);
		}
	}

	qrcode::bwindex(_BW, ext, B_cxx);
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

	//illumination
	Eigen::MatrixXd Org;
	Eigen::VectorXd Bmap,Wmap;
	igl::embree::EmbreeIntersector ei;
	std::vector<int> map;
	Eigen::VectorXd depth,depth1;
	Eigen::RowVectorXd centriod;
	qrcode::illumin_origin(V_fin, F_fin, mode, 300, 100, centriod,Org);
	qrcode::illuminate_map(ei,V_fin, F_fin, Org, Pb, Bmap);
	qrcode::illuminate_map(ei,V_fin, F_fin, Org, Pw, Wmap);
	int number = 0;
	for (int i = 0; i < Org.rows(); i++) {
		if (Wmap(i) == 1.0&& Bmap(i) <= 0.8) {
			map.push_back(i);
			number++;
		}
			
	}
	cout << "Number of candidate point:" << number << endl;
	depth.resize(map.size());
	depth1.resize(map.size());
	Eigen::MatrixXd Color(Wmap.rows(), 3);
	igl::jet(Wmap, true, Color);
	//viewer.data.clear();
	viewer.core.point_size = 5;
	viewer.data.set_points(Org, Color);
	
	for (int k = 0; k < map.size(); k++) {
		double seq = 1.0;
		int black = Pb.rows();
		Eigen::MatrixXd Light, V_qr1, V_fin1, th_crv1, Nb;
		Light.setOnes(BW.rows() - 1, BW.cols() - 1);
		V_qr1 = V_qr;
		V_fin1 = V_fin;
		th_crv1 = th_crv;
		Eigen::VectorXd _map;
		Eigen::VectorXd origin = Org.row(map[k]);
		cout << origin.transpose() << endl;
		bool stop = true;
		int scale = 1;
		qrcode::is_light(ei,V_fin, F_fin, origin, Pb, _map);
		int index = 0;
		for (int i = 0; i < BW.rows() - 1; i++) {
			for (int j = 0; j < BW.cols() - 1; j++) {
				if (BW(i, j) > 0) {
					for (int m = 0; m < scale; m++) {
						for (int n = 0; n < scale; n++) {
							Light(i*scale + m, j*scale + n) = _map(index);
							if (_map(index) != 0) {
								th_crv1(i*scale + m, j*scale + n) += c;
								stop = false;
							}
							index++;
						}
					}
				}
			}
		}
		//carve down
		qrcode::curve_down(V_uncrv, D, Src, Dir, th[0], wht_num, mul, th_crv1, V_qr1);
		//Remesh
		V_fin1.block(V_rest.rows(), 0, V_qr1.rows(), 3) << V_qr1;
		qrcode::pre_black_normal(BW, Src, Dir, th[0], th_crv1, scale, black, Pb, Nb);
		while (!stop)
		{
			seq += 1.0;
			stop = true;
			qrcode::is_light(ei,V_fin1, F_fin, origin, Pb, _map);
			int index = 0;
			for (int i = 0; i < BW.rows() - 1; i++) {
				for (int j = 0; j < BW.cols() - 1; j++) {
					if (BW(i, j) > 0) {
						for (int m = 0; m < scale; m++) {
							for (int n = 0; n < scale; n++) {
								Light(i*scale + m, j*scale + n) = _map(index);
								if (_map(index) != 0) {
									th_crv1(i*scale + m, j*scale + n) += c;
									stop = false;
								}
								index++;
							}
						}
					}
				}
			}
			//carve down
			qrcode::curve_down(V_uncrv, D, Src, Dir, th[0], wht_num, mul, th_crv1, V_qr1);
			//Remesh
			V_fin1.block(V_rest.rows(), 0, V_qr.rows(), 3) << V_qr1;
			qrcode::pre_black_normal(BW, Src, Dir, th[0], th_crv1, scale, black, Pb, Nb);
		}

		depth(k) = seq;
		//depth1(k) = (th_crv1.array() - th_crv1.mean()).matrix().norm();
		th_crv1.resize(0, 0);
		V_qr1.resize(0, 0);
		V_fin1.resize(0, 0);
		_map.resize(0);

	}  
	int pos;
	/*depth = (depth.array() - depth.minCoeff()).matrix() / (depth.maxCoeff() - depth.minCoeff())*0.5;
	depth1 = (depth1.array() - depth1.minCoeff()).matrix() / (depth1.maxCoeff() - depth1.minCoeff())*0.5;
	depth += depth1;*/
	for (int i = 0; i < depth.rows(); i++) {
		if (depth(i) < 2)
			depth(i) = 1000.0;
	}
	depth.minCoeff(&pos);

	cout << "===========================================Optimal result========================================" << endl;
	igl::matlab::MatlabWorkspace mw;
	int black1 = Pb.rows();
	int count = 0;
	Eigen::MatrixXd Light, V_qr1, V_fin1, th_crv1, Nb1;
	Light.setOnes(BW.rows() - 1, BW.cols() - 1);
	V_qr1 = V_qr;
	V_fin1 = V_fin;
	th_crv1 = th_crv;
	cout << th_crv.maxCoeff() << endl;
	Eigen::VectorXd _map;
	Eigen::VectorXd origin = Org.row(map[pos]);
    Eigen::RowVectorXd direction=(origin.transpose()-centriod).normalized();
	cout << asin(direction(2)) << endl;
	bool stop = true;
	qrcode::is_light(ei, V_fin, F_fin, origin, Pb, _map);
	int index = 0;
	for (int i = 0; i < BW.rows() - 1; i++) {
		for (int j = 0; j < BW.cols() - 1; j++) {
			if (BW(i, j) > 0) {
				for (int m = 0; m < scale; m++) {
					for (int n = 0; n < scale; n++) {
						Light(i*scale + m, j*scale + n) = _map(index);
						if (_map(index) != 0) {
							th_crv1(i*scale + m, j*scale + n) += c;
							stop = false;
						}
						index++;
					}
				}
			}
		}
	}
	//carve down
	mw.save(Light, "L");
	mw.write("result/Experiment_" + to_string(count) + ".mat");
	qrcode::curve_down(V_uncrv, D, Src, Dir, th[0], wht_num, mul, th_crv1, V_qr1);
	//Remesh
	V_fin1.block(V_rest.rows(), 0, V_qr1.rows(), 3) << V_qr1;
	qrcode::pre_black_normal(BW, Src, Dir, th[0], th_crv1, scale, black1, Pb, Nb1);
	igl::writeOBJ("result/Experiment_" + to_string(count) + ".obj", V_fin1, F_fin);
	while (!stop)
	{
		count++;
		stop = true;
		qrcode::is_light(ei, V_fin1, F_fin, origin, Pb, _map);
		int index = 0;
		for (int i = 0; i < BW.rows() - 1; i++) {
			for (int j = 0; j < BW.cols() - 1; j++) {
				if (BW(i, j) > 0) {
					for (int m = 0; m < scale; m++) {
						for (int n = 0; n < scale; n++) {
							Light(i*scale + m, j*scale + n) = _map(index);
							if (_map(index) != 0) {
								th_crv1(i*scale + m, j*scale + n) += c;
								stop = false;
							}
							index++;
						}
					}
				}
			}
		}
		cout << th_crv1.maxCoeff() << endl;
		//carve down
		qrcode::curve_down(V_uncrv, D, Src, Dir, th[0], wht_num, mul, th_crv1, V_qr1);
		//Remesh
		V_fin1.block(V_rest.rows(), 0, V_qr.rows(), 3) << V_qr1;
		mw.save(Light, "L");
		mw.write("result/Experiment_" + to_string(count) + ".mat");
		igl::writeOBJ("result/Experiment_" + to_string(count) + ".obj", V_fin1, F_fin);
		qrcode::pre_black_normal(BW, Src, Dir, th[0], th_crv1, scale, black1, Pb, Nb);
	}
	
	Eigen::VectorXd Ar;
	Eigen::MatrixXd Vis;
	Eigen::MatrixXi G;
	qrcode::ambient_occlusion(V_fin1, F_fin, Pw, Nw, 5000, Sw);
	//cout << Sw << endl;

	qrcode::ambient_occlusion(V_fin1, F_fin, Pb, Nb, meshes, Ar, Sb);
	//cout << Sb << endl;
	Vis.setZero((BW.rows() - 1)*scale, (BW.cols() - 1)*scale);
	Eigen::MatrixXd Are;
	Are.setOnes((BW.rows() - 1)*scale, (BW.cols() - 1)*scale);
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
						Are(x, y) = Ar(indexb);
						indexb++;
					}
				}
			}
		}
	}
	
	qrcode::vis2gray(engine, Vis, G);
	mw.save(Vis, "AV");
	mw.save(Are, "A");
	mw.save(G, "G");
	mw.write("Ambient.mat");
	V_fin = V_fin1;
	th_crv1.resize(0, 0);
	V_qr1.resize(0, 0);
	V_fin1.resize(0, 0);
	_map.resize(0);
	ei.global_deinit();


	/*const auto & light = [&V_fin,&F_fin,&Org,&Pb,& map,&BW,&c,&V_rest,
		&V_uncrv, &D, &Src, &Dir, &th, &wht_num, &mul, &th_crv, &V_qr,&depth](const int p)->bool {
		int black = Pb.rows();
		Eigen::MatrixXd Light,V_qr1,V_fin1,th_crv1,Nb;
		Light.setOnes(BW.rows()-1,BW.cols()-1);
		V_qr1 = V_qr;
		V_fin1 = V_fin;
		th_crv1 = th_crv;
		Eigen::VectorXd _map;
		Eigen::VectorXd origin = Org.row(map[p]);
		cout << origin.transpose() << endl;
		bool stop = true;
		int scale = 1;
		qrcode::is_light(V_fin, F_fin, origin, Pb, _map);
		int index = 0;
		for (int i = 0; i < BW.rows() - 1; i++) {
			for (int j = 0; j < BW.cols() - 1; j++) {
				if (BW(i, j) > 0) {
					for (int m = 0; m < scale; m++) {
						for (int n = 0; n < scale; n++) {
							Light(i*scale + m, j*scale + n) = _map(index);
							if (_map(index) != 0) {
								th_crv1(i*scale + m, j*scale + n) += c;
								stop = false;
							}
							index++;
						}
					}
				}
			}
		}
		//carve down
		qrcode::curve_down(V_uncrv, D, Src, Dir, th[0], wht_num, mul, th_crv1, V_qr1);
		//Remesh
		V_fin1.block(V_rest.rows(), 0, V_qr1.rows(), 3) << V_qr1;
		qrcode::pre_black_normal(BW, Src, Dir, th[0], th_crv1, scale, black, Pb, Nb);
		while (!stop)
		{
			stop = true;
			qrcode::is_light(V_fin1, F_fin, origin, Pb, _map);
			int index = 0;
			for (int i = 0; i < BW.rows() - 1; i++) {
				for (int j = 0; j < BW.cols() - 1; j++) {
					if (BW(i, j) > 0) {
						for (int m = 0; m < scale; m++) {
							for (int n = 0; n < scale; n++) {
								Light(i*scale + m, j*scale + n) = _map(index);
								if (_map(index) != 0) {
									th_crv1(i*scale + m, j*scale + n) += c;
									stop = false;
								}
								index++;
							}
						}
					}
				}
			}
			//carve down
			qrcode::curve_down(V_uncrv, D, Src, Dir, th[0], wht_num, mul, th_crv1, V_qr1);
			//Remesh
			V_fin.block(V_rest.rows(), 0, V_qr.rows(), 3) << V_qr1;
			qrcode::pre_black_normal(BW, Src, Dir, th[0], th_crv1, scale, black, Pb, Nb);
		}
		
		depth(p) = th_crv1.mean();
		th_crv1.resize(0, 0);
		V_qr1.resize(0, 0);
		V_fin1.resize(0, 0);
		_map.resize(0);
		return false;
	};
	int n = map.size();
	igl::parallel_for(n, light,1000);*/

	/*int pr;
	double dpt = depth.minCoeff(&pr);
	cout << dpt << endl;
*/



}
