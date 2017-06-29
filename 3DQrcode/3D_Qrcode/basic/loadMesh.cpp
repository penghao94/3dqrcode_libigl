#include "loadMesh.h"
#include "igl/file_dialog_open.h"
bool qrcode::loadMesh(igl::viewer::Viewer & viewer, Eigen::MatrixXd & V, Eigen::MatrixXi & F)
{
	std::string input = "";
	input = igl::file_dialog_open();
	const char* inputFile=input.c_str();
	if (input != "")
	{
		viewer.data.clear();
		viewer.load_mesh_from_file(inputFile);
		V = viewer.data.V;
		F = viewer.data.F;
	}
	return true;
}


