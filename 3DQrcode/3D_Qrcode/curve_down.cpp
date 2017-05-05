#include "curve_down.h"
#include <iostream>
using namespace std;
bool qrcode::curve_down(Eigen::MatrixXd & V, Eigen::MatrixXd & D, Eigen::MatrixXf & Src, Eigen::MatrixXf & Dir, Eigen::MatrixXd & T, const int wht_num, const int mul, Eigen::MatrixXd & addT, Eigen::MatrixXd & _V)
{
	_V.resize(V.rows(), 3);
	_V = V;
	int b_idx = 0;
	double max = 0;
	int row = D.rows()*mul;
	int col = D.cols()*mul;
	for (int i = 0; i < D.rows(); i++) {
		for (int j = 0; j < D.cols(); j++) {
			if (D(i, j) == 1.0&&i<D.rows()-1&&j<D.rows()-1) {
				int m = i*mul;
				int n = j*mul;
				_V.row(4 * b_idx + 0 + wht_num) <<(Src.row(m*col+n) + (T(m,n) + addT(i, j))*Dir.row(m*col+n)).cast<double>();
				_V.row(4 * b_idx + 1 + wht_num) <<(Src.row((m+mul)*col+n)+ (T(m+mul, n) + addT(i, j))*Dir.row((m + mul)*col + n)).cast<double>();
				_V.row(4 * b_idx + 2 + wht_num) <<(Src.row(m*col + n+mul)+ (T(m, n+mul) + addT(i, j))*Dir.row(m*col + n + mul)).cast<double>();
				_V.row(4 * b_idx + 3 + wht_num) <<(Src.row((m + mul)*col + n+mul) + (T(m+mul,n+mul) + addT(i, j))*Dir.row((m + mul)*col + n + mul)).cast<double>();
				if (-addT(i + 1, j + 1)*Dir((m + mul)*col + n + mul, 2) > max)
					max = -addT(i + 1, j + 1)*Dir((m + mul)*col + n + mul, 2);
				b_idx++;
			}
		}
	}
	cout << "Thickness:" << max << endl;
	return false;
}
