#include "readData.h"
#include <igl\file_dialog_open.h>
bool qrcode::readData(Eigen::MatrixXi & D)
{
	std::string input ="";
	input = igl::file_dialog_open();
	if (input != "") {
		readData(input, D);
		return true;
	}else
		return false;
}

bool qrcode::readData(const std::string file, Eigen::MatrixXi & D)
{
	int cols, rows, n;
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>R, G, B, A;
	unsigned char *data = stbi_load(file.c_str(), &cols, &rows, &n, 4);
	if (data == NULL) {
		return false;
	}
	R.resize(cols, rows);
	G.resize(cols, rows);
	B.resize(cols, rows);
	A.resize(cols, rows);
	D.resize(cols, rows);
	for (unsigned i = 0; i < rows; ++i) {
		for (unsigned j = 0; j < cols; ++j) {
			R(j, rows - 1 - i) = data[4 * (j + cols * i) + 0];
			G(j, rows - 1 - i) = data[4 * (j + cols * i) + 1];
			B(j, rows - 1 - i) = data[4 * (j + cols * i) + 2];
			A(j, rows - 1 - i) = data[4 * (j + cols * i) + 3];
		}
	}

	stbi_image_free(data);
	Eigen::MatrixXd temp = 0.3*R.cast<double>() + 0.59*G.cast<double>() + 0.11*B.cast<double>();
	for (int i = 0; i < temp.rows(); i++) {
		for (int j = 0; j < temp.cols(); j++) {
			D(i, j) = (temp(i, j) > 0) ? 1 : 0;
		}
	}
	//D = R.cast<int>();
	R.resize(0,0);
	G.resize(0,0);
	B.resize(0,0);
	A.resize(0,0);
	return true;
}
