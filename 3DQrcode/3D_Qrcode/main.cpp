/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * main.cpp  2017/03/13 15:43
 * TODO:
 *
*/

/*
Self-definition function 
*/
#include "basic\loadMesh.h"
#include "basic\Savemesh.h"
#include "basic\readData.h"
#include "basic\printPNG.h"
#include "qrcodeGenerator.h"
#include "img_to_mesh.h"
#include "cutMesh.h"
#include "trianglate.h"
#include "curve_down.h"
#include "display.h"
#include "test.h"
#include "bwlabel.h"
#include "mesh.h"
#include "visible_mesh.h"
#include "pre_pixel_normal.h"
#include "ambient_occlusion.h"
#include "illuminate_map.h"
#include "optimization.h"
#include "watermark.h"
#include "direction_light.h"
 
/*
Calling function
*/
#include <Eigen/core>
#include <igl/viewer/ViewerCore.h>
#include <nanogui/formhelper.h>
#include <nanogui/screen.h>
#include <igl/unique.h>
#include <igl/Timer.h>
#include <igl/readOFF.h>
#include <igl/matlab/matlabinterface.h>
#include <igl/matlab/MatlabWorkspace.h>
#include <igl/unique.h>
#include <igl/jet.h>
#include <igl/PI.h>
#include <igl/png/writePNG.h>
#include <thread>
#include <chrono>
int main(int argc, char *argv[])
{
	// Initiate viewer, timer and setting 
	igl::viewer::Viewer viewer;
	igl::Timer timer;
	Engine *engine;
	igl::matlab::mlinit(&engine);
	igl::matlab::MatlabWorkspace mw;
	viewer.core.show_lines = false;
	
	/*
	Set global parameters
	*/
	//Primary model
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	Eigen::MatrixXd C;
	int scale = 0;
	int num = 0;
	Eigen::MatrixXd D;
	Eigen::MatrixXi D_img;
	Eigen::MatrixXd func;
	//Parameters of qrcode image to mesh
	int acc = 1;					//accuracy of projection
	Eigen::MatrixXi F_hit;			//face id hit by ray
	Eigen::MatrixXd V_uncrv;		// vertex matrix uncarved
	Eigen::MatrixXi F_qr;
	Eigen::MatrixXd C_qr;
	Eigen::MatrixXi E_qr;
	Eigen::MatrixXd H_qr;
	Eigen::MatrixXf Src, Dir;		//source direction of uncarved vertex
	std::vector<Eigen::MatrixXd> th;				//than of uncarved vertex
	Eigen::MatrixXd V_pxl;			
	int wht_num;					//number of white block
	//Parameters of carve mesh down
	int mul;
	double depth=0.005;
	Eigen::MatrixXd th_crv;         //carved than
	Eigen::MatrixXd V_qr; 
	Eigen::MatrixXd T;
	//Parameters of cut mesh
	Eigen::MatrixXd V_rest;
	Eigen::MatrixXi F_rest;
	Eigen::MatrixXi E_rest;
	//Parameters of triangulate
	Eigen::Matrix4f mode,model;
	float zoom;
	Eigen::MatrixXi F_tri;
	//Parameters of final model
	Eigen::MatrixXd V_fin;
	Eigen::MatrixXi F_fin;
	Eigen::MatrixXd C_fin;
	//Parameters of calculate area
	//Output of bwlabel
	Eigen::MatrixXd BW;

	//Output of bwindex
	vector<Eigen::MatrixXd> B_cnn;
	vector<Eigen::MatrixXi> B_cxx;
	vector<Eigen::MatrixXi> B_cii;
	//Output of cut Plane
	double minZ, t;
	Eigen::MatrixXd Box;
	vector<vector<Eigen::MatrixXd>> B_mdl;
	vector<vector<Eigen::MatrixXd>> B_md;
	//Output of upperpoint
	Eigen::VectorXi upnt;
	Eigen::VectorXi ufct;
	int v_num;
	int f_num;
	int is_white=0;
	//parameter of subdivision
	Eigen::MatrixXd Vs;
	std::vector<Eigen::MatrixXi> Fs;
	std::vector<Eigen::MatrixXi>S;
	//Output of visibility
	Eigen::MatrixXd Vis;
	Eigen::RowVectorXd centr;

	/*direction light*/

	double latitude = 45., longitude = 0.;
	const auto radian = [](double angle)->double {return angle / 180 * igl::PI; };
	double distance = 10;
	/*D.resize(7, 7);
	D << 0, 0, 0, 0, 0, 0, 0,
		0, 1, 1, 1, 1, 0, 0,
		0, 1, 1, 1, 1, 0, 0,
		0, 1, 1, 1, 1, 0, 0,
		0, 1, 1, 1, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0;*/
	qrcode::test(D, T);
	//qrcode::writePNG("F:/Graphics/git/3dqrcode_libigl/3DQrcode/3D_Qrcode/images/qrcode.png", D, 1);
	scale = 1;
	// UI Design
	viewer.callback_init = [&](igl::viewer::Viewer& viewer)
	{

		// Add an additional menu window
		viewer.ngui->addWindow(Eigen::Vector2i(220, 15), "I/O Operator");

		// Add new group
		viewer.ngui->addGroup("Load & Save");

		// Add a button
		viewer.ngui->addButton("Load Mesh", [&]() {
			viewer.data.clear();
			qrcode::loadMesh(viewer, V, F);
			
			C.resize(F.rows(), 3);
			C << Eigen::RowVector3d(1.0, 1.0, 1.0).replicate(F.rows(), 1);
			viewer.data.set_colors(C);
		});

		// Add a button
		viewer.ngui->addButton("Load image", [&]() {
			qrcode::readData(D_img);
			scale = 1;
			D = D_img.transpose().cast<double>();
		});
		viewer.ngui->addButton("Load qrcode", [&]() {
			scale = 1;
			num=qrcode::readData(D);
			
			//num = qrcode::readData(engine, D, func);
		});
		// Add a button
		viewer.ngui->addButton("Save Mesh", [&]() {
			string str = igl::file_dialog_save();
			igl::writeOBJ(str, V_fin, F_fin);

		});

		viewer.ngui->addButton("Save PNG", [&]() {
			qrcode::printPNG(viewer);

		});

		viewer.ngui->addGroup("Qr code Operator");
		viewer.ngui->addVariable("Depth", depth);
		viewer.ngui->addVariable("white", is_white);
		viewer.ngui->addButton("Image to mesh", [&]() {
			timer.start();
			mode = viewer.core.model;
			zoom = viewer.core.model_zoom*viewer.core.camera_zoom;
			if (V.rows() != 0 && D.rows() != 0) {
				wht_num = qrcode::img_to_sep_mesh(viewer, V, F, D, scale, acc, F_hit, V_uncrv, F_qr, C_qr, E_qr, H_qr, Src, Dir, th,V_pxl);
			}
			//th_crv = (T.array()*(0.25 / 8 / abs(Dir(int(Dir.rows() / 2), 2)))).matrix();
			viewer.data.clear();
			viewer.data.set_mesh(V_uncrv, F_qr);
			viewer.data.set_colors(C_qr);
			cout << "Image to mesh time = " << timer.getElapsedTime() << endl;
		});
		viewer.ngui->addButton("Display", [&]() {
			viewer.data.clear();
			qrcode::display(V, F, C, F_hit, V_uncrv, F_qr, C_qr, V_fin, F_fin, C_fin);
			qrcode::cutMesh(V, F, F_hit, V_rest, F_rest, E_rest);
			mul = scale*acc;
			viewer.data.set_mesh(V_fin, F_fin);
			viewer.data.set_colors(C_fin);
			V_fin.resize(0, 0);
			F_fin.resize(0, 0);
			C_fin.resize(0, 0);
			viewer.data.set_face_based(true);
			
		});
		viewer.ngui->addButton("Carved Model", [&]() {
			th_crv.resize(D.rows(), D.cols());
			//_crv.setConstant(depth ); 
			th_crv.setConstant(depth / abs(Dir(int(Dir.rows() / 2), 2)));
			timer.start();
			viewer.data.clear();
			
			qrcode::curve_down(V_uncrv, D, Src, Dir, th[0], wht_num,mul,th_crv, V_qr);
		
			viewer.data.set_mesh(V_qr, F_qr);
			viewer.data.set_face_based(true);
			cout << "Carve model time = " << timer.getElapsedTime() << endl;
			
		});
		viewer.ngui->addButton("Merge", [&]() {
			viewer.data.clear();
			timer.start();
			qrcode::tranglate(V_rest, E_rest, V_qr, E_qr, H_qr,mode, F_tri);
			V_fin.resize(V_rest.rows() + V_qr.rows(), 3);
			F_fin.resize(F_rest.rows() + F_qr.rows() + F_tri.rows(), 3);
			C_fin.resize(F_rest.rows() + F_qr.rows() + F_tri.rows(), 3);
			V_fin.block(0, 0, V_rest.rows(), 3) << V_rest;
			V_fin.block(V_rest.rows(), 0, V_qr.rows(), 3) << V_qr;
			F_fin.block(0, 0, F_rest.rows(), 3) << F_rest;
			F_fin.block(F_rest.rows(), 0, F_qr.rows(), 3) << (F_qr.array() + V_rest.rows()).matrix();
			F_fin.block(F_rest.rows() + F_qr.rows(), 0, F_tri.rows(), 3) << F_tri;
			viewer.data.set_face_based(true);
			viewer.data.set_mesh(V_fin, F_fin);
			B_mdl.clear();
			cout << "Model vertex: " << V_fin.rows() << endl << "Model facet: " << F_fin.rows() << endl;
			cout << "Merge time = " << timer.getElapsedTime() << endl;
		});


		viewer.ngui->addButton("Calculate area(A)", [&]() {   
			timer.start();
			Eigen::MatrixXd Pb, Nb, Pw, Nw,_BW;
			Eigen::VectorXd Sw,Sb,Ar;
			
			
			
			qrcode::bwlabel(engine, D, 4, BW);
			_BW.setZero((BW.rows() - 1) / num + 1, (BW.rows() - 1) / num + 1);
			for (int i = 0; i < _BW.rows() - 1; i++) {
				for (int j = 0; j < _BW.cols() - 1; j++) {
					_BW(i, j) = BW(num*i, num*j);
				}
			}
			qrcode::bwindex(_BW,num,B_cxx);
			std::vector<qrcode::Mesh> meshes = qrcode::visible_mesh(BW, B_cxx, V_pxl);
			double Area=0;
			for (int i = 0; i < meshes.size(); i++) {
				for (int j = 0; j < meshes[i].F.rows(); j++) {
					Eigen::Vector3d a, b, c;
					a = meshes[i].V.row(meshes[i].F(j, 0)).transpose();
					b = meshes[i].V.row(meshes[i].F(j, 1)).transpose();
					c = meshes[i].V.row(meshes[i].F(j, 2)).transpose();
					Area += (a - b).cross(b - c).norm();
				}
			}
			
			cout<<"Area:" << Area << endl;
			igl::matlab::mleval(&engine, "clc,clear");
			igl::matlab::mlsetmatrix(&engine, "W", BW);
			igl::matlab::mleval(&engine, "[a,b]=size(W)");
			igl::matlab::mleval(&engine, "W=W(1:(a-1),1:(b-1))");
			igl::matlab::mleval(&engine, "w=length(find(W==0))");
			int white = igl::matlab::mlgetscalar(&engine, "w");
			int black = pow(BW.cols() - 1, 2) - white;
			qrcode::pre_black_normal(BW, Src, Dir, th[0], th_crv, scale, black, Pb, Nb);
			qrcode::pre_white_normal(BW, V_pxl, scale, white, Pw, Nw);
			
			qrcode::ambient_occlusion(V_fin, F_fin, Pw, Nw, 5000, Sw);
			//cout << Sw << endl;
			
			qrcode::ambient_occlusion(V_fin, F_fin,  Pb, Nb, meshes,Ar,Sb);
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
			mw.save(Vis, "AV");
			mw.save(Are, "A");
			mw.write("Ambient.mat");
			cout << "Area time = " << timer.getElapsedTime() << endl;
			Pb.resize(0, 0);
			Sb.resize(0, 0);
			Nw.resize(0, 0);
			Nb.resize(0, 0);
			_BW.resize(0, 0);
			Sw.resize(0);
			Sb.resize(0);
			meshes.clear();
			meshes.swap(std::vector<qrcode::Mesh>());
		});


		viewer.ngui->addButton("Illuminate map", [&]() {
			timer.start();
			Eigen::MatrixXd Pb, Nb, Pw, Nw,Org;
			Eigen::VectorXd map;
			qrcode::bwlabel(engine, D, 4, BW);
			qrcode::bwindex(BW, B_cxx);
			igl::matlab::mleval(&engine, "clc,clear");
			igl::matlab::mlsetmatrix(&engine, "W", BW);
			igl::matlab::mleval(&engine, "[a,b]=size(W)");
			igl::matlab::mleval(&engine, "W=W(1:(a-1),1:(b-1))");
			igl::matlab::mleval(&engine, "w=length(find(W==0))");
			int white = igl::matlab::mlgetscalar(&engine, "w");
			int black = pow(BW.cols() - 1, 2) - white;
			qrcode::illumin_origin(V, F, mode, 10000, 10,centr, Org);
			if (is_white==0){
				qrcode::pre_black_normal(BW, Src, Dir, th[0], th_crv, scale, black, Pb, Nb);
				//qrcode::illuminate_map(V_fin, F_fin, Org, Pb, map);
			} 
			
			else if(is_white==1){
				qrcode::pre_white_normal(BW, V_pxl, scale, white, Pw, Nw);
				//qrcode::illuminate_map(V_fin, F_fin, Org, Pw, map);
			}
			Eigen::MatrixXd Color(map.rows(), 3);
			igl::jet(map, true, Color);
			//viewer.data.clear();
			viewer.core.point_size = 5;
			viewer.data.set_points(Org, Color);
			cout << "illuminate time = " << timer.getElapsedTime() << endl;
		});


		viewer.ngui->addButton("Optimization(A)", [&]() {
			viewer.data.clear();
			qrcode::optimization(engine, D,func, mode, wht_num, mul,num ,V_uncrv, F_qr, C_qr, E_qr, H_qr, Src, Dir, th, V_pxl, V_rest, F_rest, E_rest, V_fin, F_fin);
			viewer.data.set_face_based(true);
			viewer.data.set_mesh(V_fin,F_fin);
		});
		viewer.ngui->addButton("Optimization(D)", [&]() {
			qrcode::watermark(viewer,engine, D, mode, wht_num, mul, num, V_uncrv, F_qr, C_qr, E_qr, H_qr, Src, Dir, th, V_pxl, V_rest, F_rest, E_rest, V_fin, F_fin);
			viewer.data.clear();
			viewer.data.set_mesh(V_fin, F_fin);
		});

		viewer.ngui->addVariable("latitude", latitude);
		viewer.ngui->addVariable("longitude", longitude);
		viewer.ngui->addVariable("distance", distance);
		viewer.ngui->addButton("Direction light", [&]() {
			Eigen::Vector4d position,position1;
			double latitude1 = latitude - 15.0;
			position << cos(radian(latitude))*cos(radian(longitude)), cos(radian(latitude))*sin(radian(longitude)), sin(radian(latitude)),distance;
			position1 << cos(radian(latitude1))*cos(radian(longitude)), cos(radian(latitude1))*sin(radian(longitude)), sin(radian(latitude1)), distance;
			qrcode::direction_light(viewer, engine, position,position1, D, mode,zoom, wht_num, mul, num, V_uncrv, F_qr, C_qr, E_qr, H_qr, Src, Dir, th, V_pxl, V_rest, F_rest, E_rest, V_fin, F_fin);
		});
		// Generate menu
		viewer.screen->performLayout();

		return false;
	};
	// Launch the viewer
	viewer.launch();

}
