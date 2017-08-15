#include "qrcodeGenerator.h"
#include "qrcodeModifier.h"
#include "igl/png/writePNG.h"
#include <igl/unique.h>
#include "../bwlabel.h"
#include "../visibility/halfedge.h"
bool qrcode::qrCodeGenerator(std::string text, const qrcodegen::QrCode::Ecc & errColLvl, int mask,int border, Eigen::MatrixXd & Q)
{
	const char*str = text.c_str();
	const qrcodegen::QrCode _qr = qrcodegen::QrCode::encodeText(str, errColLvl);
	qrcodegen::QrCode qr = qrcodegen::QrCode(_qr, mask);
	qrcode::qrcodeModefier(qr);
	Q.setZero(2 * border + qr.size + 1, 2 * border + qr.size + 1);
	for (int i = -border; i < qr.size + border; i++) {
		for (int j = -border; j < qr.size + border; j++) {
			//1 is black block
			Q(i+border, j+border) = (qr.getModule(j, i) == 1 ? 1.0 : 0.0);
		}
	}
	return true;
}

bool qrcode::qrCodeGenerator(std::string text, int errColLvl, int mask, int border, Eigen::MatrixXd & Q, Eigen::MatrixXd & F)
{
	const char*str = text.c_str();
	const qrcodegen::QrCode::Ecc *newEcl; 
	switch (errColLvl)
	{
	case 0:
		newEcl = &qrcodegen::QrCode::Ecc::LOW;
		break;
	case 1:
		newEcl = &qrcodegen::QrCode::Ecc::MEDIUM;
		break;
	case 2:
		newEcl = &qrcodegen::QrCode::Ecc::QUARTILE;
		break;
	case 3:
		newEcl = &qrcodegen::QrCode::Ecc::HIGH;
		break;
	}
		qrcodegen::QrCode _qr = qrcodegen::QrCode::encodeText(str, *newEcl);
		qrcodegen::QrCode qr = qrcodegen::QrCode(_qr, mask);
		qrcode::qrcodeModefier(qr);
		Q.setZero(2 * border + qr.size + 1, 2 * border + qr.size + 1);
		F.setZero(2 * border + qr.size + 1, 2 * border + qr.size + 1);
		for (int i = -border; i < qr.size + border; i++) {
			for (int j = -border; j < qr.size + border; j++) {
				//1 is black block
				Q(i + border, j + border) = (qr.getModule(j, i) == 1 ? 1.0 : 0.0);
				F(i + border, j + border) = (qr.getFunctionModule(j, i) == 1 ? 1.0 : 0.0);
			}
		}
	return true;
}

bool qrcode::writePNG(std::string file, Eigen::MatrixXd & Q, int scale)
{
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R,G, B, A;
	R.resize((Q.rows()-1)*scale, (Q.cols()-1)*scale);
	G.resize((Q.rows() - 1)*scale, (Q.cols() - 1)*scale);
	B.resize((Q.rows() - 1)*scale, (Q.cols() - 1)*scale);
	A.resize((Q.rows() - 1)*scale, (Q.cols() - 1)*scale);
	for (int i = 0; i < Q.rows()-1; i++) {
		for (int j = 0; j < Q.cols()-1; j++) {
			for (int x = 0; x < scale; x++) {
				for (int y = 0; y < scale; y++) {
					if (Q( Q.cols()-2-j,i) == 0) {
						R(i*scale + x, j*scale + y) = 255;
						G(i*scale + x, j*scale + y) = 255;
						B(i*scale + x, j*scale + y) = 255;
					}
					else {
						R(i*scale + x, j*scale + y) = 0;
						G(i*scale + x, j*scale + y) = 0;
						B(i*scale + x, j*scale + y) = 0;
					}
					A(i*scale + x, j*scale + y) = 255;
				}
			}
		}
	}
	igl::png::writePNG(R, G, B, A, file);
	return true;
}

void qrcode::writePNG(Engine *engine, std::string file, int scale, Eigen::MatrixXd & D, Eigen::MatrixXd & F)
{
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A; //RGB and alpha

	R.resize((D.rows() - 1)*scale, (D.cols() - 1)*scale);
	G.resize((D.rows() - 1)*scale, (D.cols() - 1)*scale);
	B.resize((D.rows() - 1)*scale, (D.cols() - 1)*scale);
	A.resize((D.rows() - 1)*scale, (D.cols() - 1)*scale);

	//Origin PNG data

	for (int i = 0; i < D.rows() - 1; i++) {
		for (int j = 0; j < D.cols() - 1; j++) {
			for (int x = 0; x < scale; x++) {
				for (int y = 0; y < scale; y++) {
					R(i*scale + x, j*scale + y) = D(D.cols() - 2 - j, i) == 0 ? 204 : F(D.cols() - 2 - j, i) == 0 ? 174 : 153;
						
					G(i*scale + x, j*scale + y) = D(D.cols() - 2 - j, i) == 0 ? 204 : F(D.cols() - 2 - j, i) == 0 ? 174 : 153;

					B(i*scale + x, j*scale + y) = D(D.cols() - 2 - j, i) == 0 ? 204 : F(D.cols() - 2 - j, i) == 0 ? 174 : 153;
					
				A(i*scale + x, j*scale + y) = 255;
				}
			}
		}
	}
	// exchange PNG data
	Eigen::MatrixXd BW; //bwlabel of data

	qrcode::bwlabel(engine, D, 4, BW);

	Eigen::MatrixXi _BW, _E, E(0,2);// data and edge

	int col = (BW.rows() - 1)*scale;	//cols of PNG

	igl::unique(BW.cast<int>(), _BW);

	int index = _BW.rows() - 1; _BW.resize(0, 0);//number of unique bw label

	std::vector<std::vector<int>> P(index); // point of data
	for (int i = 0; i < BW.rows(); i++) {
		for (int j = 0; j < BW.cols(); j++) {
			if (round(BW(i, j)) != 0&&F(i,j)==0.0) 	P[round(BW(i, j)) - 1].push_back(i*scale*col + j*scale);
		}
	}

	for (int i = 0; i < index; i++) {
		qrcode::eList *elist = new qrcode::eList();

		Eigen::MatrixXi EA(4*P[i].size(),2),ER;
		for (int j = 0; j < P[i].size(); j++) {

			int a = P[i][j];
			int b = P[i][j] + scale*col;
			int c = P[i][j] + scale;
			int d = P[i][j] + scale*col+scale;

			elist->add(a, b, 0);
			EA.row(j * 4) << a, b;
			elist->add(b, d, 0);
			EA.row(j * 4 + 1) << b, d;
			elist->add(d, c, 0);
			EA.row(j * 4 + 2) << d, c;
			elist->add(c, a, 0);
			EA.row(j * 4 + 3) << c, a;
		}
		
		elist->matrix(_E);
		delete elist;
		_E = _E.block(0, 0, _E.rows(), 2);

		igl::matlab::mlsetmatrix(&engine,"E",_E);
		igl::matlab::mlsetmatrix(&engine, "EA", EA);
		igl::matlab::mleval(&engine, "ER=setdiff(EA,E,'rows');");
		igl::matlab::mlgetmatrix(&engine, "ER", ER);
		E.conservativeResize(E.rows() + ER.rows(), 2);
		E.block(E.rows() - ER.rows(), 0, ER.rows(), 2)=ER;

	}
	
	for (int i = 0; i < E.rows(); i++) {
		int sx = E(i, 0) / col; int sy = E(i, 0) % col;
		int dx = E(i, 1) / col; int dy = E(i, 1) % col;
		//std::cout << sx << "  " << sy << " " << dx << "  " << dy << std::endl;
		for (int j = 0; j < scale; j++) {
			for (int k =0 ; k < scale; k++) {
				int x = col - sx - k*(dx - sx) / scale;
				int y = sy + j*(dy - sy) / scale;
				R(y-1, x-1) = 195;
				G(y-1, x-1) = 195;
				B(y-1, x-1) = 195;
			}
		}
	}
	
	igl::png::writePNG(R, G, B, A, file);
}
