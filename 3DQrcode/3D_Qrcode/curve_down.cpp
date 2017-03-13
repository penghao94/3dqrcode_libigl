#include "curve_down.h"
#include <iostream>
using namespace std;
bool qrcode::curve_down(Eigen::MatrixXd & V, Eigen::MatrixXd & D, Eigen::MatrixXf & Src, Eigen::MatrixXf & Dir, Eigen::MatrixXd & T, const int wht_num, Eigen::MatrixXd & addT, Eigen::MatrixXd & _V)
{
	_V.resize(V.rows(), 3);
	_V = V;
	int b_idx = 0;

	for (int i = 0; i < D.rows(); i++) {
		for (int j = 0; j < D.cols(); j++) {
			if (D(i, j) == 1.0&&i<D.rows()-1&&j<D.rows()-1) {
				_V.row(4 * b_idx + 0 + wht_num) << (Src.row(i*D.cols() + j) + (T(i, j) + addT(i, j))*Dir.row(i*D.cols() + j)).cast<double>();
				_V.row(4 * b_idx + 1 + wht_num) <<( Src.row((i+1)*D.cols() + j)+ (T(i + 1, j) + addT(i + 1, j))*Dir.row((i + 1)*D.cols() + j)).cast<double>();
				_V.row(4 * b_idx + 2 + wht_num) << (Src.row(i*D.cols() + j+1)+ (T(i, j+1) + addT(i, j+1))*Dir.row(i*D.cols() + j+1)).cast<double>();
				_V.row(4 * b_idx + 3 + wht_num) <<( Src.row((i + 1)*D.cols() + j+1) + (T(i+1, j+1) + addT(i+1, j+1))*Dir.row((i+1)*D.cols() + j+1)).cast<double>();
				b_idx++;
			}
		}
	}
	return false;
}
