#include "SaveMesh.h"

#include "igl/file_dialog_open.h"
#include "igl/file_dialog_save.h"
#include <Eigen/Dense>
#include <igl/readOFF.h>
#include <igl/writeOFF.h>
#include <igl/viewer/Viewer.h>
#include <nanogui/formhelper.h>
#include <nanogui/screen.h>
#include <igl/unproject_onto_mesh.h>
//#include <igl/jet.h>

#include <iostream>

using namespace std;
using namespace igl;

int main(int argc, char *argv[])
{
  // Initiate viewer
  igl::viewer::Viewer viewer;

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  Eigen::MatrixXd C;
  //V.resize(0, 0);
  //F.resize(0, 0);
  //C.resize(0, 0);

  // UI Design
  viewer.callback_init = [&](igl::viewer::Viewer& viewer)
  {
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
			  qrcode::SaveMesh(output,viewer,viewer.data);
			  //viewer.save_mesh_to_file(output);
		  }
	  });

	  // Generate menu
	  viewer.screen->performLayout();

	  return false;
  };

  // Select Target Region


  viewer.callback_mouse_down =
	  [&V, &F, &C](igl::viewer::Viewer& viewer, int, int)->bool
  {
	  int fid;
	  Eigen::Vector3f bc;
	  // Cast a ray in the view direction starting from the mouse position
	  double x = viewer.current_mouse_x;
	  double y = viewer.core.viewport(3) - viewer.current_mouse_y;
	  if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core.view * viewer.core.model,
		  viewer.core.proj, viewer.core.viewport, V, F, fid, bc))
	  {
		  // paint hit red
		  C.row(fid) << 1, 0, 0;
		  cout << fid << endl;
		  viewer.data.set_colors(C);
		  return true;
	  }
	  return false;
  };

  // Launch the viewer
  viewer.launch();

}
