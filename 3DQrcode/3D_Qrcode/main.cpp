#include "main.h"
#include "SaveMesh.h"
#include "Select.h"
#include "LoadQRCode.h"
#include "readData.h"

#include "igl/file_dialog_open.h"
#include "igl/file_dialog_save.h"
#include <Eigen/Dense>
//#include <igl/readOFF.h>
//#include <igl/writeOFF.h>
#include <igl/viewer/ViewerCore.h>
#include <nanogui/formhelper.h>
#include <nanogui/screen.h>
//#include <igl/jet.h>

using namespace igl;

int main(int argc, char *argv[])
{
  // Initiate viewer
  igl::viewer::Viewer viewer;
  viewer.core.background_color << 1.0f, 1.0f, 1.0f, 1.0f;

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  Eigen::MatrixXd C;

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
		  Eigen::MatrixXi D;
		  qrcode::readData(D);
		  cout << D << endl;
	  });

	  // Generate menu
	  viewer.screen->performLayout();

	  return false;
  };

  // Select Target Region

  viewer.callback_mouse_down =
	  [&V, &F, &C](igl::viewer::Viewer& viewer, int, int)->bool
  {
	  qrcode::select(viewer, V, F, C);
	  return false;
  };
  
  //GLuint iTex;
  //qrcode::LoadT8("images/IRC_version1_1.bmp", iTex);
  //glTranslatef(0, 0, -10);
  //qrcode::texture(iTex);
  //qrcode::tPic(3.0f);

  // Launch the viewer
  viewer.launch();

}