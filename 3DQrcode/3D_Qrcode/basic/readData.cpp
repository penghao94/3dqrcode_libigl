
#include "readData.h"
#include<vector>
#include <igl\file_dialog_open.h>
#include "qrcodeGenerator.h"
#include "../bwlabel.h"
#include "../visible_mesh.h"
#include "../2D_visible.h"
#include "../mesh.h"
#include <iostream>
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
	if (input != ""){
		Eigen::MatrixXd _D;
		int scale=readData(input, _D);
		D.setZero((_D.rows() - 1)*scale + 1, (_D.cols() - 1)*scale + 1);
		for(int i=0;i<_D.rows()-1;i++){
			for(int j=0;j<_D.cols()-1;j++){
				for(int m=0;m<scale;m++){
					for(int n=0;n<scale;n++){
						D(i*scale + m, j*scale + n) = _D(i, j);
					}
				}
			}
		}
		return scale;
	}
	else
		return 0;
}

int qrcode::readData(Engine *engine, Eigen::MatrixXd & D, Eigen::MatrixXd & isF)
{
	std::string input = "";
	input = igl::file_dialog_open();
	if (input != ""){
		Eigen::MatrixXd _D;
		int scale=readData(input, engine, _D,isF);
		D.setOnes((_D.rows() - 1)*scale + 1, (_D.cols() - 1)*scale + 1);
		for(int i=0;i<_D.rows()-1;i++){
			for(int j=0;j<_D.cols()-1;j++){
				for(int m=0;m<scale;m++){
					for(int n=0;n<scale;n++){
						D(i*scale+m, j*scale+n) = _D(i, j);
					}
				}
			}
		}
		return scale;
	}
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
		fclose(in);
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

int qrcode::readData(const std::string file, Engine *engine, Eigen::MatrixXd & D, Eigen::MatrixXd & F)
{
	using namespace std;
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
		fscanf(in, "%d %d %d %d\n", &errColLvl, &mask, &border, &scale);
		fscanf(in, "%s\n", &text);
		fclose(in);
		std::string str = text;
		Eigen::MatrixXd BW;
		vector<Eigen::MatrixXi> B_cxx;
		if (mask == -1) {
			for (int i = 0; i < 8; i++) {
				qrCodeGenerator(str, errColLvl, i, border, D, F);
				qrcode::bwlabel(engine, D, 4, BW);
				qrcode::bwindex(BW, B_cxx);
				double Area;
				std::vector<qrcode::Mesh> meshes;
				std::vector<Eigen::Vector2d> B;
				const int col = BW.cols()*scale;
				qrcode::Mesh mesh;
				Eigen::MatrixXd H(0, 2);
				for (int i = 0; i < BW.rows() - 1; i++) {
					for (int j = 0; j < BW.cols() - 1; j++) {
						for (int m = 0; m < scale; m++) {
							for (int n = 0; n < scale; n++) {
								if (BW(i, j) > 0) {
									int x = i*scale + m;
									int y = j*scale + n;
									qrcode::lightRegion(Eigen::RowVector2d(x + 0.5, y + 0.5), B_cxx[BW(i, j) - 1], scale, BW.cols(), B);
									Eigen::MatrixXd V_qr(B.size()+1, 3);
									Eigen::MatrixXi f;
									for (int k = 0; k < B.size(); k++) {
										V_qr.row(k) << B[k](0),B[k](1),0;
									}
									V_qr.row(V_qr.rows() - 1) <<x + 0.5, y + 0.5,0;
									mesh.V = V_qr;
									f.resize(B.size(), 3);
									for (int k = 0; k < B.size(); k++)
										f.row(k) << k, B.size(), (k + 1) % B.size();
									assert(B.size() + 1 == V_qr.rows() && f.rows() > 0);
									mesh.F = f;
									meshes.push_back(mesh);
									B.swap(std::vector<Eigen::Vector2d>());
									V_qr.resize(0, 0);
									f.resize(0, 0);
									mesh.V.resize(0, 0);
									mesh.F.resize(0, 0);
								}
							}
						}
					}
				}
				for (int i = 0; i < meshes.size(); i++) {
					for (int j = 0; j < meshes[i].F.rows(); j++) {
						Eigen::Vector3d a, b, c;
						a = meshes[i].V.row(meshes[i].F(j, 0)).transpose();
						b = meshes[i].V.row(meshes[i].F(j, 1)).transpose();
						c = meshes[i].V.row(meshes[i].F(j, 2)).transpose();
						Area += (a - b).cross(b - c).norm();
					}
				}
			}
		}else
			qrCodeGenerator(str, errColLvl, mask, border, D,F);
		return scale;
	}

}
