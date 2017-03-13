#include "saveResult.h"

bool qrcode::saveResult(igl::viewer::Viewer & viewer)
{
	std::string output = "";
	output = igl::file_dialog_save();
	saveResult(viewer, output);
	return true;
}

bool qrcode::saveResult(igl::viewer::Viewer & viewer, const std::string file)
{

	// Allocate temporary buffers for 1280x800 image
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(1280, 800);
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(1280, 800);
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(1280, 800);
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(1280, 800);
	if (file != "") {
		// Draw the scene in the buffers
		viewer.core.draw_buffer(viewer.data, viewer.opengl, false, R, G, B, A);
		// Save it with a format of png
		igl::png::writePNG(R, G, B, A, file);
		return true;
	}
	else
		return false;

}
