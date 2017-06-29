#include "watermark.h"
#include "curve_down.h"
#include "trianglate.h"
#include "bwlabel.h"
#include "gaussianKernel.h"
#include "mesh.h"
#include "visible_mesh.h"
#include "pre_pixel_normal.h"
#include "illuminate_map.h"
#include <igl/Timer.h>
#include <igl/parallel_for.h>
void qrcode::watermark(Engine * engine, Eigen::MatrixXd & D, Eigen::Matrix4f & mode, int wht_num, int mul, int ext, 
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
	double c =0.5* step / abs(Dir(a*D.cols() + b, 2));
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
	int scale = th[0].cols() / BW.cols();

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
	std::vector<int> map;
	Eigen::VectorXd depth;
	qrcode::illumin_origin(V_fin, F_fin, mode, 10000, 10, Org);
	qrcode::illuminate_map(V_fin, F_fin, Org, Pb, Bmap);
	qrcode::illuminate_map(V_fin, F_fin, Org, Pw, Wmap);

	for (int i = 0; i < Org.rows(); i++) {
		if (Wmap(i) == 1.0&& Bmap(i) <=0.2)
			map.push_back(i);
	}
	depth.resize(map.size());

	/*for (int k = 0; k < map.size(); k++) {
		Eigen::MatrixXd Light;
		Eigen::VectorXd _map;
		Light.setZero(D.rows(), D.cols());
		Eigen::VectorXd origin = Org.row(map[k]);
		bool stop = true;

		qrcode::is_light(V_fin, F_fin, origin, Pb, _map);
		int index = 0;
		for (int i = 0; i < D.rows() - 1; i++) {
			for (int j = 0; j < D.cols() - 1; j++) {
				if (BW(i, j) == 0) {
					for (int m = 0; m < scale; m++) {
						for (int n = 0; n < scale; n++) {
							if (_map(index) != 0) {
								th_crv(i*scale + m, j*scale + n) += c;
								stop = false;
							}
							index++;
						}
					}
				}
			}
		}
		//carve down
		qrcode::curve_down(V_uncrv, D, Src, Dir, th[0], wht_num, mul, th_crv, V_qr);
		//Remesh
		V_fin.block(V_rest.rows(), 0, V_qr.rows(), 3) << V_qr;

		while (!stop)
		{
			stop = true;
			qrcode::is_light(V_fin, F_fin, origin, Pb, _map);
			int index = 0;
			for (int i = 0; i < D.rows() - 1; i++) {
				for (int j = 0; j < D.cols() - 1; j++) {
					if (BW(i, j) == 0) {
						for (int m = 0; m < scale; m++) {
							for (int n = 0; n < scale; n++) {
								if (_map(index) != 0) {
									th_crv(i*scale + m, j*scale + n) += c;
									stop = false;
								}
								index++;
							}
						}
					}
				}
			}
			//carve down
			qrcode::curve_down(V_uncrv, D, Src, Dir, th[0], wht_num, mul, th_crv, V_qr);
			//Remesh
			V_fin.block(V_rest.rows(), 0, V_qr.rows(), 3) << V_qr;
		}


	}  
*/


	const auto & light = [&V_fin,&F_fin,&Org,&Pb,& map,&BW,&c,&V_rest,
		&V_uncrv, &D, &Src, &Dir, &th, &wht_num, &mul, &th_crv, &V_qr,&depth](const int p) {
		Eigen::MatrixXd Light,V_qr1,V_fin1,th_crv1;
		V_qr1 = V_qr;
		V_fin1 = V_fin;
		th_crv1 = th_crv;
		Eigen::VectorXd _map;
		Light.setZero(D.rows(), D.cols());
		Eigen::VectorXd origin = Org.row(map[p]);
		bool stop = true;
		int scale = 1;
		qrcode::is_light(V_fin, F_fin, origin, Pb, _map);
		int index = 0;
		for (int i = 0; i < D.rows() - 1; i++) {
			for (int j = 0; j < D.cols() - 1; j++) {
				if (BW(i, j) == 0) {
					for (int m = 0; m < scale; m++) {
						for (int n = 0; n < scale; n++) {
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

		while (!stop)
		{
			stop = true;
			qrcode::is_light(V_fin1, F_fin, origin, Pb, _map);
			int index = 0;
			for (int i = 0; i < D.rows() - 1; i++) {
				for (int j = 0; j < D.cols() - 1; j++) {
					if (BW(i, j) == 0) {
						for (int m = 0; m < scale; m++) {
							for (int n = 0; n < scale; n++) {
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
		}
		depth(p) = th_crv1.mean();
	};
	int n = map.size();
	igl::parallel_for(n, light,1000);



}
