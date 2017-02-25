#include "main.h"
#include "SaveMesh.h"
#include "Select.h"
#include "LoadQRCode.h"
#include "readData.h"
#include "img_to_mesh.h"
#include "igl/file_dialog_open.h"
#include "igl/file_dialog_save.h"
#include <Eigen/Dense>
//#include <igl/readOFF.h>
//#include <igl/writeOFF.h>
#include <igl/viewer/ViewerCore.h>
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <nanogui/formhelper.h>
#include <nanogui/screen.h>
#include "igl/Timer.h"
//#include <igl/jet.h>

using namespace igl;

int main(int argc, char *argv[])
{
  // Initiate viewer
  igl::viewer::Viewer viewer;
  igl::Timer timer;
  viewer.core.show_lines = false;
  //viewer.core.background_color << 1.0f, 1.0f, 1.0f, 1.0f;

  Eigen::MatrixXd V,_V;
  Eigen::MatrixXi F,_F;
  Eigen::MatrixXd C;
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
		  //viewer.data.clear();
		  string InputFile = "";
		  const char *input;
		  InputFile = igl::file_dialog_open();
		  input = InputFile.c_str();
		  if (InputFile != "")
		  {
			  viewer.data.clear();
			  viewer.load_mesh_from_file(input);
			  //igl::readOFF(InputFile, V, F);
			  V = viewer.data.V;
			  F = viewer.data.F;
			  // Initialize white
			  C = Eigen::MatrixXd::Constant(F.rows(), 3, 1);
			  viewer.data.set_colors(C);
		  }
	  });

	  // Add a button
	  viewer.ngui->addButton("Save Mesh", [&]() {
		  string OutputFile = "";
		  const char *output;
		  OutputFile = igl::file_dialog_save();
		  output = OutputFile.c_str();
		  if (OutputFile != "")
		  {
			  //cout << OutputFile;
			  qrcode::saveMesh(output,viewer,viewer.data);
			  //viewer.save_mesh_to_file(output);
		  }
	  });

	  // Add a button
	  viewer.ngui->addButton("Load	QRCode", [&]() {
		  /*string InputQRCode = "";
		  InputQRCode = igl::file_dialog_open();
		  if (InputQRCode != "")
		  {
			  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;
			  qrcode::readPNG(InputQRCode, R, G, B, A);
			  //cout << R << G << B << A;
			  qrcode::loadQRCode(viewer, R, G, B);
		  }*/
		 
		  qrcode::readData(D);
		  //cout << D << endl;
	  });
	  viewer.ngui->addButton("Test Project", [&]() {
		  //igl::readOFF(InputFile, V, F);
		  viewer.data.clear();
		  Eigen::MatrixXi fid;
		  Eigen::MatrixXd  _C;
		  /*D.resize(4, 4);
		  D << 1, 1, 1, 1,
			  1, 0, 0, 1,
			  1, 0, 0, 1,
			  1, 1, 1, 1;
		  D.resize(2, 2);
		  D << 1, 0,
			  0, 0;*/
		  //cout << D << endl;
		  timer.start();
		  qrcode::img_to_mesh(viewer, V, F, D, fid, _V, _F, _C);
		  cout << "time = " << timer.getElapsedTime() << endl;
		  //cout << "V" << _V << endl << "F" << _F <<endl;
		  //C= Eigen::MatrixXd::Constant(_V.rows(), 3, 0);
		  //viewer.data.set_points(_V,C);

		  viewer.data.set_mesh(_V,_F);
		  viewer.data.set_colors(_C);

		  //cout << viewer.core.model << endl;
		  //cout << fid << endl;
		  //cout << "_V" << endl << _V << endl;
		  //cout <<"D"<<endl<< _D << endl;
		 // cout << "F" << endl << _F << endl;
		 // cout << "C" << endl << _C << endl;
	  });

	  viewer.ngui->addButton("Merge	QRCode", [&]() {

		  Eigen::MatrixXd VC;
		  Eigen::MatrixXi FC;
		  Eigen::VectorXi J;
		  igl::MeshBooleanType boolean_type(igl::MESH_BOOLEAN_TYPE_UNION);
		  igl::copyleft::cgal::mesh_boolean(V, F, _V, _F, boolean_type, VC, FC, J);

	  });

	  // Generate menu
	  viewer.screen->performLayout();

	  return false;
  };

  // Select Target Region

  /*viewer.callback_mouse_down =
	  [&V, &F, &C](igl::viewer::Viewer& viewer, int, int)->bool
  {
	  qrcode::select(viewer, V, F, C);
	  return false;
  };*/

  // Launch the viewer
  viewer.launch();

}