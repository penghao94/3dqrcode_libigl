#include "pre_pixel_normal.h"
#include <Eigen/dense>
void qrcode::pre_black_normal(Eigen::MatrixXd & BW, Eigen::MatrixXf & Src, Eigen::MatrixXf & Dir, Eigen::MatrixXd & th, Eigen::MatrixXd & th_crv, int scale, int num_black, Eigen::MatrixXd & P, Eigen::MatrixXd & N)
{
	using namespace std;
	using namespace Eigen;
	Eigen::Vector3d a, b, c, d, n1, n2;
	const int col = BW.cols()*scale;
	P.resize(num_black*scale*scale, 3);
	N.resize(num_black*scale*scale, 3);
	int index = 0;
	for (int i = 0; i < BW.rows() - 1; i++) {
		for (int j = 0; j < BW.cols() - 1; j++) {
			if (BW(i, j) > 0) {
				a = Src.row(i*scale*col + j*scale).cast<double>() + Dir.row(i*scale*col + j*scale).cast<double>()*(th(i*scale, j*scale) + th_crv(i, j));
				b = Src.row((i + 1)*scale*col + j*scale).cast<double>() + Dir.row((i + 1)*scale*col + j*scale).cast<double>()*(th((i + 1)*scale, j*scale) + th_crv(i, j));
				d = Src.row((i + 1)*scale*col + (j + 1)*scale).cast<double>() + Dir.row((i + 1)*scale*col + (j + 1)*scale).cast<double>()*(th((i + 1)*scale, (j + 1)*scale) + th_crv(i, j));
				c = Src.row(i*scale*col + (j + 1)*scale).cast<double>() + Dir.row(i*scale*col + (j + 1)*scale).cast<double>()*(th(i*scale, (j + 1)*scale) + th_crv(i, j));
				n1 = (b - a).cross(c - b);
				n2 = (d - b).cross(c - d);
				for (int m = 0; m < scale; m++) {
					for (int n = 0; n < scale; n++) {
						int x = i*scale + m;
						int y = j*scale + n;
						P.row(index) << (Src.row((x+1)*th.cols() + y).cast<double>() + Dir.row((x+1)*th.cols() + y).cast<double>()*(th(x+1, y) + th_crv(i, j)) +
							Src.row(x *th.cols() + y + 1).cast<double>() + Dir.row(x*th.cols() + y + 1).cast<double>()*(th(x, y + 1) + th_crv(i, j))
							) / 2;
						if (m > n)
							N.row(index) = n1.normalized().transpose();
						else if (m == n)
							N.row(index) = ((n1 + n2) / 2).normalized().transpose();
						else if (m < n)
							N.row(index) = n2.normalized().transpose();
						index++;
					}
				}

			}
		}
	}
}

void qrcode::pre_white_normal(Eigen::MatrixXd & BW, Eigen::MatrixXd & V_pxl, int scale, int num_white,Eigen::MatrixXd & P, Eigen::MatrixXd & N)
{
	using namespace std;
	using namespace Eigen;
	Eigen::Vector3d a, b, c, d, n1, n2;
	const int col = BW.cols()*scale;
	P.resize(num_white*scale*scale, 3);
	N.resize(num_white*scale*scale, 3);
	int index = 0;
	for (int i = 0; i < BW.rows() - 1; i++) {
		for (int j = 0; j < BW.cols() - 1; j++) {
			if (BW(i, j) == 0) {
				a = V_pxl.row(i*scale*col + j*scale).transpose();
				b = V_pxl.row((i + 1)*scale*col + j*scale).transpose();
				d = V_pxl.row((i + 1)*scale*col + (j + 1)*scale).transpose();
				c = V_pxl.row(i*scale*col + (j + 1)*scale).transpose();
				n1 = (b - a).cross(c - b);
				n2 = (d-b).cross(c-b);
			for (int m = 0; m < scale; m++) {
				for (int n = 0; n < scale; n++) {
						int x = i*scale + m;
						int y = j*scale + n;
						 P.row(index)<< (V_pxl.row((x+1)*col + y)+ V_pxl.row(x*col + y + 1)) / 2;
						 if (m > n)
							 N.row(index) = n1.normalized().transpose();
						 else if (m == n)
							 N.row(index) = ((n1 + n2) / 2).normalized().transpose();
						 else if (m < n)
							 N.row(index) = n2.normalized().transpose();
						 index++;
					}
				}
			
			}
		}	}
}

void qrcode::pre_white_normal2(Eigen::MatrixXd & BW, Eigen::MatrixXd & V_pxl, int scale, int num_white, Eigen::MatrixXd & P, Eigen::MatrixXd & N)
{
	using namespace std;
	using namespace Eigen;
	Eigen::Vector3d a, b, c, d, n1, n2;
	const int col = BW.cols()*scale;
	P.resize(num_white*scale*scale, 3);
	N.resize(num_white*scale*scale, 3);
	int index = 0;
	for (int i = 0; i < BW.rows() - 1; i++) {
		for (int j = 0; j < BW.cols() - 1; j++) {
			if (BW(i, j) > 0) {
				a = V_pxl.row(i*scale*col + j*scale).transpose();
				b = V_pxl.row((i + 1)*scale*col + j*scale).transpose();
				c = V_pxl.row((i + 1)*scale*col + (j + 1)*scale).transpose();
				d = V_pxl.row(i*scale*col + (j + 1)*scale).transpose();
				n1 = (b - a).cross(c - b);
				n2 = (c - a).cross(d - c);
				for (int m = 0; m < scale; m++) {
					for (int n = 0; n < scale; n++) {
						int x = i*scale + m;
						int y = j*scale + n;
						P.row(index) << (V_pxl.row(x*col + y) + V_pxl.row((x + 1)*col + y + 1)) / 2;
						if (m > n)
							N.row(index) = n1.normalized().transpose();
						else if (m == n)
							N.row(index) = ((n1 + n2) / 2).normalized().transpose();
						else if (m < n)
							N.row(index) = n2.normalized().transpose();
						index++;
					}
				}

			}
		}
	}
}
