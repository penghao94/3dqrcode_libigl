#include "LoadQRCode.h"


bool qrcode::readPNG(const std::string png_file, 
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& R, 
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& G, 
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& B, 
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& A)
{
	int cols, rows, n;
	unsigned char *data = stbi_load(png_file.c_str(), &cols, &rows, &n, 4);
	if (data == NULL) {
		return false;
	}

	R.resize(cols, rows);
	G.resize(cols, rows);
	B.resize(cols, rows);
	A.resize(cols, rows);

	for (unsigned i = 0; i < rows; ++i) {
		for (unsigned j = 0; j < cols; ++j) {
			R(j, rows - 1 - i) = data[4 * (j + cols * i) + 0];
			G(j, rows - 1 - i) = data[4 * (j + cols * i) + 1];
			B(j, rows - 1 - i) = data[4 * (j + cols * i) + 2];
			A(j, rows - 1 - i) = data[4 * (j + cols * i) + 3];
		}
	}

	stbi_image_free(data);

	return true;
}

bool qrcode::loadQRCode(igl::viewer::Viewer& viewer,
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& R,
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& G,
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& B)
{
	Eigen::MatrixXd V(4, 3);
	V <<
	-0.5, -0.5, 0,
	0.5, -0.5, 0,
	0.5, 0.5, 0,
	-0.5, 0.5, 0;
	Eigen::MatrixXi F(2, 3);
	F <<
	0, 1, 2,
	2, 3, 0;
	Eigen::MatrixXd UV(4, 2);
	UV <<
	0, 0,
	1, 0,
	1, 1,
	0, 1;
	viewer.data.clear();
	viewer.data.set_mesh(V, F);
	viewer.data.set_uv(UV);
	viewer.core.show_texture = true;
	viewer.data.set_texture(R, G, B);
	return false;
}
