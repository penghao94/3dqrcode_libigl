
#include "readData.h"
#include <igl\file_dialog_open.h>
#include "qrcodeGenerator.h"
int qrcode::readData(Eigen::MatrixXi & D)
{
	std::string input ="";
	int scale = 0;
	input = igl::file_dialog_open();
	if (input != "") {
		return readData(input, D);
	}else
		return scale ;
}

int qrcode::readData(Eigen::MatrixXd & D)
{
	std::string input = "";
	input = igl::file_dialog_open();
	if (input != "")
		return readData(input, D);
	else
		return 0;
}

int qrcode::readData(const std::string file, Eigen::MatrixXi & D)
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
			R(i,j) = data[4 * (i + cols * j) + 0];
			G(i, j) = data[4 * (i + cols * j) + 1];
			B(i, j) = data[4 * (i + cols * j) + 2];
			A(i, j) = data[4 * (i + cols * j) + 3];
		}
	}

	stbi_image_free(data);
	Eigen::MatrixXd temp = 0.3*R.cast<double>() + 0.59*G.cast<double>() + 0.11*B.cast<double>();
	for (int i = 0; i < temp.rows(); i++) {
		for (int j = 0; j < temp.cols(); j++) {
			D(i, j) = (temp(i, j) > 0) ? 0:1;
		}
	}
	R.resize(0,0);
	G.resize(0,0);
	B.resize(0,0);
	A.resize(0,0);
	return D.rows();
}

int qrcode::readData(const std::string file, Eigen::MatrixXd & D)
{
	int mask = 0;
	int border = 1;
	int errColLvl = 0;
	int scale = 1;
	char text[1024];
	const char* input = file.c_str();
	FILE* in = fopen(input, "r");
	if (in == (FILE*)NULL) {
		printf("Cannot open the qrcode info file...");
		return 0;
	}
	else {
		fscanf(in, "%d %d %d %d\n", &errColLvl, &mask, &border,&scale);
		fscanf(in, "%s\n", &text);
		std::string str = text;
		switch (errColLvl)
		{
		case 0:
			qrCodeGenerator(str, qrcodegen::QrCode::Ecc::LOW, mask, border, D);
			writePNG("qrcode.png",D, 5);
			
			break;
		case 1:
			qrCodeGenerator(str, qrcodegen::QrCode::Ecc::MEDIUM, mask, border, D);
			writePNG("qrcode.png", D, 5);
			break;
		case 2:
			qrCodeGenerator(str, qrcodegen::QrCode::Ecc::QUARTILE, mask, border, D);
			writePNG("qrcode.png", D, 5);
			break;
		case 3:
			qrCodeGenerator(str, qrcodegen::QrCode::Ecc::HIGH, mask, border, D);
			writePNG("qrcode.png", D, 5);
			break;
		}
		return scale;
	}
	
}
