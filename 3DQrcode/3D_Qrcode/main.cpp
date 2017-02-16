#include <igl/readOFF.h>
#include <igl/writeOFF.h>
#include <igl/viewer/Viewer.h>
#include <nanogui/formhelper.h>
#include <nanogui/screen.h>
#include "igl/file_dialog_open.h"
#include "igl/file_dialog_save.h"
//#include <igl/jet.h>
#include <iostream>

using namespace std;
//#include "tutorial_shared_path.h"

Eigen::MatrixXd V;
Eigen::MatrixXi F;
//Eigen::MatrixXd C;

int main(int argc, char *argv[])
{
  // Initiate viewer
  igl::viewer::Viewer viewer;

  viewer.callback_init = [&](igl::viewer::Viewer& viewer)
  {
	  // Add new group
	  viewer.ngui->addGroup("Load & Save");

	  // Add a button
	  viewer.ngui->addButton("Load Mesh", [&]() { 
		  viewer.data.clear();
		  string InputFile = "";
		  const char *input;
		  InputFile = igl::file_dialog_open();
		  input = InputFile.c_str();
		  if (InputFile != "")
		  {
			  viewer.load_mesh_from_file(input);
			  //igl::readOFF(InputFile, V, F);
			  viewer.data.set_mesh(V, F);
		  }
		  
	  });

	  viewer.ngui->addButton("Save Mesh", [&]() {
		  string OutputFile = "";
		  const char *output;
		  OutputFile = igl::file_dialog_save();
		  output = OutputFile.c_str();
		  if (OutputFile != "")
		  {
			  viewer.save_mesh_to_file(output);
		  }
	  });

	  // Generate menu
	  viewer.screen->performLayout();

	  return false;
  };

  // Launch the viewer
  viewer.launch();

}
