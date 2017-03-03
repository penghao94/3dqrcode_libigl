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
//#include <igl/jet.h>

using namespace igl;

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
		viewer.ngui->addButton("QR unproject", [&]() {
			viewer.data.clear();
			Eigen::MatrixXi  fid;
			timer.start();
			qrcode::img_to_mesh(viewer, V, F, D, fid, _V, _F, _C,_E,_H);
			cout << "time = " << timer.getElapsedTime() << endl;
			timer.start();
			qrcode::cutMesh(V, F, fid, rest_V, rest_F, rest_E);
			cout << "time = " << timer.getElapsedTime() << endl;
			viewer.data.set_mesh(_V, _F);
			viewer.data.set_colors(_C);
		});

		viewer.ngui->addButton("Merge	QRCode", [&]() {
			Eigen::MatrixXd Vt, H;
			Eigen::MatrixXi Ft, E;
			timer.start();
			H.resize(1, 2);
			H.row(0) << _H(0, 0), _H(0, 1);
			E = rest_E.block(0, 0, rest_E.rows(), 2);
			qrcode::tranglate(_V, _E, V, E, H, Vt, Ft);
			cout << "time = " << timer.getElapsedTime() << endl;
			viewer.data.clear();
			viewer.data.set_mesh(Vt, Ft);
		});
		viewer.ngui->addButton("Test", [&]() {
			viewer.data.clear();
			Eigen::MatrixXi fid;
			Eigen::MatrixXd Vt,H;
			Eigen::MatrixXi Ft,E;
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
			qrcode::tranglate(_V, _E, V, E, H, Vt, Ft);
			cout << "time = " << timer.getElapsedTime() << endl;
			viewer.data.clear();
			//cout << Vt << endl;
			cout << endl;
			//cout << Ft << endl;
			//viewer.data.set_mesh(Vt, Ft);
		});
		// Generate menu
		viewer.screen->performLayout();

		return false;
	};
	// Launch the viewer
	viewer.launch();

}