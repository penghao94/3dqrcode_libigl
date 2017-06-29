#include "visible_mesh.h"
#include "mesh.h"
#include "2D_visible.h"
#include "igl/triangle/triangulate.h"
#include <igl/parallel_for.h>
std::vector<qrcode::Mesh> qrcode::visible_mesh(Eigen::MatrixXd & BW, std::vector<Eigen::MatrixXi>& B_qr, Eigen::MatrixXd &V_pxl)
{
	using namespace std;
	std::vector<qrcode::Mesh> meshes;
	int scale = 1;
	const int col = (BW.cols() - 1)*scale + 1;
	vector<Eigen::RowVector2i> pos, _pos;
	for (int i = 0; i < BW.rows() - 1; i++) {
		for (int j = 0; j < BW.cols() - 1; j++) {
			for (int m = 0; m < scale; m++) {
				for (int n = 0; n < scale; n++) {
					if (BW(i, j) > 0) {
						int x = i*scale + m;
						int y = j*scale + n;
						pos.push_back(Eigen::RowVector2i(x, y));
						_pos.push_back(Eigen::RowVector2i(i, j));
					}
				}
			}
		}
	}
	meshes.resize(pos.size());
	const auto &visible = [&pos,&_pos,&BW,&B_qr,&V_pxl,&col,&meshes](const int p) {
		std::vector<Eigen::Vector2d> B;
		qrcode::Mesh mesh;
		qrcode::lightRegion(Eigen::RowVector2d(pos[p](0) + 0.5, pos[p](1) + 0.5), B_qr[BW(_pos[p](0), _pos[p](1)) - 1], 1, BW.cols(), B);
		Eigen::MatrixXd V_qr(B.size(), 3);
		Eigen::MatrixXi f;
		for (int k = 0; k < B.size(); k++) {
			int x1 = floor(B[k](1));
			int y1 = floor(B[k](0));
			int x2 = x1 + 1;
			int y2 = y1 + 1;
			V_qr.row(k) = V_pxl.row(x1*col + y1) + (B[k](1) - floor(B[k](1)))*(V_pxl.row((x1 + 1)*col + y1) - V_pxl.row(x1*col + y1)) + (B[k](0) - floor(B[k](0)))*(V_pxl.row(x1*col + y1 + 1) - V_pxl.row(x1*col + y1));
		}
		V_qr.conservativeResize(V_qr.rows() + 1, 3);
		V_qr.row(V_qr.rows() - 1) = (V_pxl.row(pos[p](0)*col + pos[p](1)) + V_pxl.row((pos[p](0) + 1)*col + pos[p](1) + 1)) / 2;
		mesh.V = V_qr;
		f.resize(B.size(), 3);
		for (int k = 0; k < B.size(); k++)
			f.row(k) << k, B.size(), (k + 1) % B.size();
		mesh.F = f;
		meshes[p]=mesh;
		B.swap(std::vector<Eigen::Vector2d>());
		V_qr.resize(0, 0);
		f.resize(0, 0);
		mesh.V.resize(0, 0);
		mesh.F.resize(0, 0);
	};
	igl::parallel_for(pos.size(), visible, 1000);
	pos.clear();
	_pos.clear();
	pos.swap(vector<Eigen::RowVector2i>());
	_pos.swap(vector<Eigen::RowVector2i>());
	return meshes;
}