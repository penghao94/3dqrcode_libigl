#ifndef LOADQRCODE_H
#define LOADQRCODE_H

#include <Eigen/Core>
#include <string>
#include <igl/png/readPNG.h>
#include <igl/viewer/Viewer.h>
#include <stb_image.h>

namespace qrcode {
	bool readPNG(const std::string png_file,
		Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& R,
		Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& G,
		Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& B,
		Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& A
	);
	bool loadQRCode(igl::viewer::Viewer& viewer, 
		Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& R,
		Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& G,
		Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& B);
}
#endif // !LOADQRCODE_H