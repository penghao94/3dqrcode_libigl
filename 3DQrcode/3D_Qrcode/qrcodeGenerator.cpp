#include "qrcodeGenerator.h"
#include "igl/png/writePNG.h"
bool qrcode::qrCodeGenerator(std::string text, const qrcodegen::QrCode::Ecc & errColLvl, int mask,int border, Eigen::MatrixXd & Q)
{
	const char*str = text.c_str();
	const qrcodegen::QrCode _qr = qrcodegen::QrCode::encodeText(str, errColLvl);
	qrcodegen::QrCode qr = qrcodegen::QrCode(_qr, mask);
	Q.resize(2 * border + qr.size, 2 * border + qr.size);
	for (int i = -border; i < qr.size + border; i++) {
		for (int j = -border; j < qr.size + border; j++) {
			Q(i+border, j+border) = (qr.getModule(j, i) == 1 ? 1.0 : 0.0);
		}
	}
	return true;
}

bool qrcode::writePNG(std::string file, Eigen::MatrixXd & Q, int scale)
{
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R,G, B, A;
	R.resize(Q.rows()*scale, Q.cols()*scale);
	G.resize(Q.rows()*scale, Q.cols()*scale);
	B.resize(Q.rows()*scale, Q.cols()*scale);
	A.resize(Q.rows()*scale, Q.cols()*scale);
	for (int i = 0; i < Q.rows(); i++) {
		for (int j = 0; j < Q.cols(); j++) {
			for (int x = 0; x < scale; x++) {
				for (int y = 0; y < scale; y++) {
					if (Q(i, Q.cols()-1-j) == 0) {
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
