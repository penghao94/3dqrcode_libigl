#include "qrcodeGenerator.h"
#include "igl/png/writePNG.h"
bool qrcode::qrCodeGenerator(std::string text, const qrcodegen::QrCode::Ecc & errColLvl, int mask,int border, Eigen::MatrixXd & Q)
{
	const char*str = text.c_str();
	const qrcodegen::QrCode _qr = qrcodegen::QrCode::encodeText(str, errColLvl);
	qrcodegen::QrCode qr = qrcodegen::QrCode(_qr, mask);
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
	if (errColLvl == 0) {
		qrcodegen::QrCode _qr = qrcodegen::QrCode::encodeText(str, qrcodegen::QrCode::Ecc::LOW);
		qrcodegen::QrCode qr = qrcodegen::QrCode(_qr, mask);
		Q.setZero(2 * border + qr.size + 1, 2 * border + qr.size + 1);
		F.setZero(2 * border + qr.size + 1, 2 * border + qr.size + 1);
		for (int i = -border; i < qr.size + border; i++) {
			for (int j = -border; j < qr.size + border; j++) {
				//1 is black block
				Q(i + border, j + border) = (qr.getModule(j, i) == 1 ? 1.0 : 0.0);
				F(i + border, j + border) = (qr.getFunctionModule(j, i) == 1 ? 1.0 : 0.0);
			}
		}
	}	
	else if (errColLvl == 1) {
		qrcodegen::QrCode _qr = qrcodegen::QrCode::encodeText(str, qrcodegen::QrCode::Ecc::MEDIUM);
		qrcodegen::QrCode qr = qrcodegen::QrCode(_qr, mask);
		Q.setZero(2 * border + qr.size + 1, 2 * border + qr.size + 1);
		F.setZero(2 * border + qr.size + 1, 2 * border + qr.size + 1);
		for (int i = -border; i < qr.size + border; i++) {
			for (int j = -border; j < qr.size + border; j++) {
				//1 is black block
				Q(i + border, j + border) = (qr.getModule(j, i) == 1 ? 1.0 : 0.0);
				F(i + border, j + border) = (qr.getFunctionModule(j, i) == 1 ? 1.0 : 0.0);
			}
		}
	}	 
	else if (errColLvl == 2) {
		 qrcodegen::QrCode _qr = qrcodegen::QrCode::encodeText(str, qrcodegen::QrCode::Ecc::QUARTILE);
		 qrcodegen::QrCode qr = qrcodegen::QrCode(_qr, mask);
		 Q.setZero(2 * border + qr.size + 1, 2 * border + qr.size + 1);
		 F.setZero(2 * border + qr.size + 1, 2 * border + qr.size + 1);
		 for (int i = -border; i < qr.size + border; i++) {
			 for (int j = -border; j < qr.size + border; j++) {
				 //1 is black block
				 Q(i + border, j + border) = (qr.getModule(j, i) == 1 ? 1.0 : 0.0);
				 F(i + border, j + border) = (qr.getFunctionModule(j, i) == 1 ? 1.0 : 0.0);
			 }
		 }
	}
	else if (errColLvl == 3) {
	qrcodegen::QrCode _qr = qrcodegen::QrCode::encodeText(str, qrcodegen::QrCode::Ecc::HIGH);
	qrcodegen::QrCode qr = qrcodegen::QrCode(_qr, mask);
	Q.setZero(2 * border + qr.size + 1, 2 * border + qr.size + 1);
	F.setZero(2 * border + qr.size + 1, 2 * border + qr.size + 1);
	for (int i = -border; i < qr.size + border; i++) {
		for (int j = -border; j < qr.size + border; j++) {
			//1 is black block
			Q(i + border, j + border) = (qr.getModule(j, i) == 1 ? 1.0 : 0.0);
			F(i + border, j + border) = (qr.getFunctionModule(j, i) == 1 ? 1.0 : 0.0);
		}
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
