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

int main(int argc, char *argv[])
{
	// Initiate viewer, timer and setting 
	igl::viewer::Viewer viewer;
	igl::Timer timer;
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
	Eigen::MatrixXd th;				//than of uncarved vertex
	Eigen::MatrixXi B_seq;			
	int wht_num;					//number of white block
	//Parameters of carve mesh down
	int mul;
	Eigen::MatrixXd th_crv;         //carved than
	Eigen::MatrixXd V_qr; 
	//Parameters of cut mesh
	Eigen::MatrixXd V_rest;
	Eigen::MatrixXi F_rest;
	Eigen::MatrixXi E_rest;
	//Parameters of triangulate
	Eigen::Matrix4f mode;
	Eigen::MatrixXi F_tri;
	//Parameters of final model
	Eigen::MatrixXd V_fin;
	Eigen::MatrixXi F_fin;
	Eigen::MatrixXd C_fin;
	//parameters of test
	//Eigen::MatrixXd d, t;
	//qrcode::test(D, t);
	//th_crv = t*0.00041;
	//scale = 2;
	//Parameters of calculate area
	Eigen::MatrixXd BW;
	vector<vector<int>> B_cnn;
	// UI Design
	viewer.callback_init = [&](igl::viewer::Viewer& viewer)
	{

		// Add an additional menu window
		viewer.ngui->addWindow(Eigen::Vector2i(220, 15), "I/O Operator");

		// Add new group
		viewer.ngui->addGroup("Load & Save");

		// Add a button
		viewer.ngui->addButton("Load Mesh", [&]() {
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

		viewer.ngui->addGroup("Qrcode Operator");

		viewer.ngui->addButton("Image to mesh", [&]() {
			timer.start();
			mode = viewer.core.model;
			mode.inverse();
			if (V.rows() != 0 && D.rows() != 0) {
				wht_num = qrcode::img_to_sep_mesh(viewer, V, F, D, scale, acc, F_hit, V_uncrv, F_qr, C_qr, E_qr, H_qr, Src, Dir, th,B_seq);
			}
			th_crv.resize(th.rows(), th.cols());
			th_crv.setConstant(0.001);
			cout << "Image to mesh time = " << timer.getElapsedTime() << endl;
		});
		viewer.ngui->addButton("Display", [&]() {
			viewer.data.clear();
			qrcode::display(V, F, C, F_hit, V_uncrv, F_qr, C_qr, V_fin, F_fin, C_fin);
			viewer.data.set_mesh(V_fin, F_fin);
			viewer.data.set_colors(C_fin);
			V_fin.resize(0, 0);
			F_fin.resize(0, 0);
			C_fin.resize(0, 0);
			viewer.data.set_face_based(true);
		});
		viewer.ngui->addButton("Carved Model", [&]() {
			timer.start();
			viewer.data.clear();
			mul = scale*acc;
			qrcode::curve_down(V_uncrv, D, Src, Dir, th, wht_num,mul,th_crv, V_qr);
			qrcode::cutMesh(V, F, F_hit, V_rest, F_rest, E_rest);
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
			cout << "Merge time = " << timer.getElapsedTime() << endl;
		});

		viewer.ngui->addButton("Calculate area", [&]() {
			qrcode::bwlabel(D, 4, BW);
			//qrcode::bwindex(BW, B_cnn);
			/*for (int i = 0; i < B_cnn.size(); i++) {
				for (int j = 0; j < B_cnn[i].size(); j++) {
					cout << B_cnn[i][j] << " ";
				}
				cout << endl;
			}*/
		});
		// Generate menu
		viewer.screen->performLayout();

		return false;
	};
	// Launch the viewer
	viewer.launch();

}
