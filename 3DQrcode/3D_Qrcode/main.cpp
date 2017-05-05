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
#include "loadMesh.h"
#include "SaveMesh.h"
#include "readData.h"
#include "img_to_mesh.h"
#include "cutMesh.h"
#include "trianglate.h"
#include "qrcodeGenerator.h"
#include "curve_down.h"
#include "printPNG.h"
#include "display.h"
#include "test.h"
#include "bwlabel.h"
#include "visibility.h"
#include "gaussianKernel.h"
#include "cut_plane.h"
#include "sphericalPolygen.h"
#include "optimization.h"
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
#include <igl/unique.h>
int main(int argc, char *argv[])
{
	// Initiate viewer, timer and setting 
	igl::viewer::Viewer viewer;
	igl::Timer timer;
	Engine *engine;
	igl::matlab::mlinit(&engine);
	viewer.core.show_lines = false;
	
	/*
	Set global parameters
	*/
	//Primary model
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	Eigen::MatrixXd C;
	int scale = 0;
	Eigen::MatrixXd D;
	Eigen::MatrixXi D_img;
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
	double depth=0.05;
	//Eigen::MatrixXd th_crv;         //carved than
	Eigen::MatrixXd V_qr; 
	Eigen::MatrixXd T;
	//Parameters of cut mesh
	Eigen::MatrixXd V_rest;
	Eigen::MatrixXi F_rest;
	Eigen::MatrixXi E_rest;
	//Parameters of triangulate
	Eigen::Matrix4f mode,model;
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
	//Output of visibility
	Eigen::MatrixXd Vis;
	
	D.resize(7, 7);
	D << 0, 0, 0, 0, 0, 0, 0,
		0, 1, 1, 1, 1, 0, 0,
		0, 1, 1, 0, 1, 0, 0,
		0, 1, 0, 1, 1, 0, 0,
		0, 0, 1, 1, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0;
	//qrcode::test(D, T);
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
			scale=qrcode::readData(D);
		});
		// Add a button
		viewer.ngui->addButton("Save Mesh", [&]() {
			qrcode::saveMesh(viewer, viewer.data);

		});

		viewer.ngui->addButton("Save PNG", [&]() {
			qrcode::printPNG(viewer);

		});

		viewer.ngui->addGroup("Qr code Operator");
		viewer.ngui->addVariable("Depth", depth);
		viewer.ngui->addButton("Image to mesh", [&]() {
			timer.start();
			mode = viewer.core.model;
			if (V.rows() != 0 && D.rows() != 0) {
				wht_num = qrcode::img_to_sep_mesh(viewer, V, F, D, scale, acc, F_hit, V_uncrv, F_qr, C_qr, E_qr, H_qr, Src, Dir, th,V_pxl);
			}
			
			//th_crv = (T.array()*(0.25 / 8 / abs(Dir(int(Dir.rows() / 2), 2)))).matrix();
			
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
		/*viewer.ngui->addButton("Carved Model", [&]() {
			//th_crv.resize(th[0].rows(), th[0].cols());
		//	th_crv.setConstant(depth / abs(Dir(int(Dir.rows() / 2), 2))); 
			timer.start();
			viewer.data.clear();
			
			//qrcode::curve_down(V_uncrv, D, Src, Dir, th[0], wht_num,mul,th_crv, V_qr);
			viewer.data.set_mesh(V_rest, F_rest);
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
			minZ = 0, t = 0;
			qrcode::cut_plane(engine,V_fin, F_fin, mode,10, B_mdl,minZ,t,Box);
			cout << "Model vertex: " << V_fin.rows() << endl << "Model facet: " << F_fin.rows() << endl;
			cout << "Merge time = " << timer.getElapsedTime() << endl;
			/ *for (int i = 0; i < B_mdl.size(); i++) {
				cout << i << endl; 
				for (int j = 0; j < B_mdl[i][0].rows(); j++) {
					Eigen::MatrixXd Eg(2, 3);
					Eg.row(0) << B_mdl[i][0].row(j);
					Eg.row(1) << B_mdl[i][1].row(j);
					igl::matlab::mlsetmatrix(&engine, "E", Eg);
					igl::matlab::mleval(&engine, "plot3(E(:,1),E(:,2),E(:,3))");
					igl::matlab::mleval(&engine, "hold on");
				}	
			}* /
			
		});

		viewer.ngui->addButton("Calculate area", [&]() {
			timer.start();
			qrcode::bwlabel(engine,D, 4, BW);
			qrcode::bwindex(BW,B_cxx);

			//qrcode::bwindex(engine,BW, scale,B_cii);
			//qrcode::upperpoint(V_fin, F_fin, mode, V_rest.rows(), wht_num, F_rest.rows(), 2 * (wht_num - D.rows() - D.cols() + 1), upnt, ufct,v_num,f_num);
			//qrcode::visibility(engine,V_pxl, Src, Dir, th, th_crv, BW, B_cxx, V_fin, F_fin, ufct, v_num, f_num, Vis);
			//cout << Vis << endl;
		//	qrcode::visibility(engine,V_pxl, Src, Dir, th[0], th_crv, BW, B_cxx, mode, minZ, t,Box, B_mdl, Vis);
			//qrcode::visibility( V_pxl, Src, Dir, th, th_crv, BW, B_cxx, mode, minZ, t, Box, B_mdl,B_md, Vis);
			cout << "Area time = " << timer.getElapsedTime() << endl;
		});*/
		viewer.ngui->addButton("Optimization", [&]() {
			viewer.data.clear();
			qrcode::optimization(engine, D, mode, wht_num, mul, V_uncrv, F_qr, C_qr, E_qr, H_qr, Src, Dir, th, V_pxl, V_rest, F_rest, E_rest, V_fin, F_fin);
			viewer.data.set_face_based(true);
			viewer.data.set_mesh(V_fin,F_fin);
		});

		
		// Generate menu
		viewer.screen->performLayout();

		return false;
	};
	// Launch the viewer
	viewer.launch();

}
