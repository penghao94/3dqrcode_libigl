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
#include <igl/unique.h>
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
  Eigen::MatrixXd C,_C;
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
		  //D = D.block(0, 0, 32, 32);
		  //cout << D << endl;
	  });
	  viewer.ngui->addButton("QR Mesh", [&]() {
		  //igl::readOFF(InputFile, V, F);
		  viewer.data.clear();
		  Eigen::MatrixXi t_F, fid;
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
		  Eigen::VectorXi Fi;
		  igl::unique(fid, Fi);
		  t_F.resize(F.rows() - Fi.size(), 3);
		  int j = 0;
		  //cout << Fi << endl;
		  for (int i = 0; i < F.rows(); i++)
		  {
			  if (i == Fi(j))
				  j++;
			  else
				  t_F.row(i - j) << F(i);
		  }
		  //cout << "V" << _V << endl << "F" << _F <<endl;
		  //C= Eigen::MatrixXd::Constant(_V.rows(), 3, 0);
		  //viewer.data.set_points(_V,C);

		  viewer.data.set_mesh(V,t_F);
		  //viewer.data.set_colors(_C);

		  //cout << viewer.core.model << endl;
		  //cout << fid << endl;
		  //cout << "_V" << endl << _V << endl;
		  //cout <<"D"<<endl<< _D << endl;
		 // cout << "F" << endl << _F << endl;
		 // cout << "C" << endl << _C << endl;
	  });

	  viewer.ngui->addButton("Merge	QRCode", [&]() {

		  viewer.data.clear();
		  Eigen::MatrixXd V_temp, V_union;
		  Eigen::MatrixXi F_temp, F_union;
		  Eigen::MatrixXd C_union(F_union.rows(), 3);
		  Eigen::VectorXi J;
		  timer.start();
		  igl::MeshBooleanType boolean_type(igl::MESH_BOOLEAN_TYPE_UNION);
		  //igl::copyleft::cgal::mesh_boolean(V, F, _V, _F, 2, V_temp, F_temp, J);
		  //igl::copyleft::cgal::mesh_boolean(V_temp, F_temp, _V, _F, igl::MESH_BOOLEAN_TYPE_UNION, V_union, F_union, J);
		  igl::copyleft::cgal::mesh_boolean( _V, _F, V, F, igl::MESH_BOOLEAN_TYPE_UNION, V_union, F_union, J);
		  cout << "time = " << timer.getElapsedTime() << endl;
		  viewer.data.set_mesh(V_union, F_union);
		  /*for (size_t f = 0; f < C.rows(); f++)
		  {
			  if (J(f) < F.rows())
			  {
				  C_union.row(f) = _C.row(f);
			  }
			  else
			  {
				  C_union.row(f) = Eigen::RowVector3d(0, 1, 0);
			  }
		  }*/

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