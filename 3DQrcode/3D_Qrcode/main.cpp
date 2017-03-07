#include "loadMesh.h"
#include "SaveMesh.h"
#include "readData.h"
#include "img_to_mesh.h"
#include "cutMesh.h"
#include "trianglate.h"
#include <Eigen/Dense>
#include <igl/viewer/ViewerCore.h>
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <nanogui/formhelper.h>
#include <nanogui/screen.h>
#include <igl/unique.h>
#include <igl/Timer.h>
#include <igl/readOFF.h>
//#include <igl/jet.h>
#include "QrCode.hpp"
#include "qrcodeGenerator.h"
using namespace igl;
static void printQr(const qrcodegen::QrCode &qr) {
	int border = 4;
	for (int y = -border; y < qr.size + border; y++) {
		for (int x = -border; x < qr.size + border; x++) {
			std::cout << (qr.getModule(x, y) == 1 ? "##" : "  ");
		}
		std::cout << std::endl;
	}
}
int main(int argc, char *argv[])
{
	// Initiate viewer
	igl::viewer::Viewer viewer;
	igl::Timer timer;
	viewer.core.show_lines = false;
	Eigen::MatrixXd V, _V,rest_V,_H;
	Eigen::MatrixXi F, _F,rest_F,rest_E,_E;
	Eigen::MatrixXd C, _C;
	Eigen::MatrixXi D;
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
		});
		// Add a button
		viewer.ngui->addButton("Load	QRCode", [&]() {
			qrcode::readData(D);
		});
		// Add a button
		viewer.ngui->addButton("Save Mesh", [&]() {
			qrcode::saveMesh(viewer, viewer.data);

		});


		viewer.ngui->addGroup("Qrcode Operator");
		viewer.ngui->addButton("QR mesh", [&]() {
			viewer.data.clear();
			Eigen::MatrixXi  fid;
			timer.start();
			qrcode::img_to_mesh(viewer, V, F, D, fid, _V, _F, _C, _E, _H);
			cout << "time = " << timer.getElapsedTime() << endl;
			timer.start();
			qrcode::cutMesh(V, F, fid, rest_V, rest_F, rest_E);
			cout << "time = " << timer.getElapsedTime() << endl;
			viewer.data.set_mesh(_V, _F);
			viewer.data.set_colors(_C);
		});
		viewer.ngui->addButton("QR unproject", [&]() {
			viewer.data.clear();
			Eigen::MatrixXi  fid;
			timer.start();
			qrcode::img_to_mesh(viewer, V, F, D, fid, _V, _F, _C,_E,_H);
			cout << "time = " << timer.getElapsedTime() << endl;
			timer.start();
			qrcode::cutMesh(V, F, fid, rest_V, rest_F, rest_E);
			cout << "time = " << timer.getElapsedTime() << endl;
			viewer.data.set_mesh(rest_V, rest_F);
			//viewer.data.set_colors(_C);
		});
		viewer.ngui->addButton("triangulation", [&]() {
			Eigen::MatrixXd Vt, H, V_all;
			Eigen::MatrixXi Ft, E, F_all;
			timer.start();
			E = rest_E.block(0, 0, rest_E.rows(), 2);
			H.resize(1, 2);
			H.row(0) << _H(0, 0), _H(0, 1);
			qrcode::tranglate(rest_V, E, _V, _E, H, Vt,Ft);
			cout << "time = " << timer.getElapsedTime() << endl;
			viewer.data.clear();
			viewer.data.set_mesh(Vt, Ft);
		});
		viewer.ngui->addButton("Merge	QRCode", [&]() {
			Eigen::MatrixXd Vt, H,V_all;
			Eigen::MatrixXi Ft, E,F_all;
			timer.start();
			E = rest_E.block(0, 0, rest_E.rows(), 2);
			H.resize(1, 2);
			H.row(0) << _H(0, 0), _H(0, 1);
			qrcode::tranglate(rest_V, E, _V, _E, H, Ft);
			cout << "time = " << timer.getElapsedTime() << endl;
			viewer.data.clear();
			V_all.resize(rest_V.rows() + _V.rows(), 3);
			F_all.resize(rest_F.rows() + _F.rows() + Ft.rows(), 3);
			V_all.block(0, 0, V.rows(), 3) << V;
			V_all.block(V.rows(), 0, _V.rows(), 3) << _V;
			F_all.block(0, 0, rest_F.rows(), 3) << rest_F;
			F_all.block(rest_F.rows(), 0, _F.rows(), 3) << (_F.array() + rest_V.rows()).matrix();
			F_all.block(rest_F.rows() + _F.rows(), 0, Ft.rows(), 3) << Ft;
			viewer.data.set_mesh(V_all, F_all);
		});
		viewer.ngui->addButton("Test", [&]() {
			viewer.data.clear();
			Eigen::MatrixXi fid;
			Eigen::MatrixXd H,V_all;
			Eigen::MatrixXi Ft,E,F_all;
			timer.start();
			igl::readOFF("F:/Graphics/git/3dqrcd_libigl/3DQrcode/3D_Qrcode/models/planexy.off", V, F);
			qrcode::readData("F:/Graphics/git/3dqrcd_libigl/3DQrcode/3D_Qrcode/images/qrcode_64.png", D);
			qrcode::img_to_mesh(viewer, V, F, D, fid, _V, _F, _C, _E, _H);
			qrcode::cutMesh(V, F, fid, rest_V, rest_F,rest_E);
			cout << "time = " << timer.getElapsedTime() << endl;
			timer.start();
			E = rest_E.block(0, 0, rest_E.rows(), 2);
			H.resize(1, 2);
			H.row(0) << _H(0, 0),_H(0,1);
			qrcode::tranglate(rest_V, E, _V, _E, H,Ft);
			cout << "time = " << timer.getElapsedTime() << endl;
			viewer.data.clear();
			V_all.resize(rest_V.rows() + _V.rows(),3);
			F_all.resize(rest_F.rows() + _F.rows()+Ft.rows(),3);
			V_all.block(0, 0, V.rows(), 3) << V;
			V_all.block(V.rows(), 0, _V.rows(), 3) << _V;
			F_all.block(0, 0, rest_F.rows(), 3) << rest_F;
			F_all.block(rest_F.rows(), 0, _F.rows(), 3) << (_F.array()+rest_V.rows()).matrix();
			F_all.block(rest_F.rows() + _F.rows(), 0, Ft.rows(), 3) << Ft;
			viewer.data.set_mesh(V_all, F_all);
		});
		viewer.ngui->addButton("Test qrcode", [&]() {
			viewer.data.clear();
			Eigen::MatrixXi fid;
			Eigen::MatrixXd H, V_all,_D;
			Eigen::MatrixXi Ft, E, F_all;
			timer.start();
			igl::readOFF("F:/Graphics/git/3dqrcd_libigl/3DQrcode/3D_Qrcode/models/planexy.off", V, F);
			int scale=qrcode::readData("F:/Graphics/git/3dqrcd_libigl/3DQrcode/3D_Qrcode/images/qrcode.txt", _D);
			qrcode::img_to_mesh(viewer, V, F, _D,scale, fid, _V, _F, _C, _E, _H);
			qrcode::cutMesh(V, F, fid, rest_V, rest_F, rest_E);
			cout << "time = " << timer.getElapsedTime() << endl;
			timer.start();
			E = rest_E.block(0, 0, rest_E.rows(), 2);
			H.resize(1, 2);
			H.row(0) << _H(0, 0), _H(0, 1);
			qrcode::tranglate(rest_V, E, _V, _E, H, Ft);
			cout << "time = " << timer.getElapsedTime() << endl;
			viewer.data.clear();
			V_all.resize(rest_V.rows() + _V.rows(), 3);
			F_all.resize(rest_F.rows() + _F.rows() + Ft.rows(), 3);
			V_all.block(0, 0, V.rows(), 3) << V;
			V_all.block(V.rows(), 0, _V.rows(), 3) << _V;
			F_all.block(0, 0, rest_F.rows(), 3) << rest_F;
			F_all.block(rest_F.rows(), 0, _F.rows(), 3) << (_F.array() + rest_V.rows()).matrix();
			F_all.block(rest_F.rows() + _F.rows(), 0, Ft.rows(), 3) << Ft;
			viewer.data.set_mesh(V_all, F_all);
		});
		// Generate menu
		viewer.screen->performLayout();

		return false;
	};
	// Launch the viewer
	viewer.launch();

}
