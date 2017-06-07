#include "visible_mesh.h"
#include "mesh.h"
#include "2D_visible.h"
#include "igl/triangle/triangulate.h"
std::vector<qrcode::Mesh> qrcode::visible_mesh(Eigen::MatrixXd & BW, std::vector<Eigen::MatrixXi>& B_qr, Eigen::MatrixXd &V_pxl, int scale)
{
	using namespace std;
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
						qrcode::lightRegion(Eigen::RowVector2d(x + 0.5, y + 0.5), B_qr[BW(i, j) - 1], scale, BW.cols(), B);
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
						V_qr.row(V_qr.rows() - 1) = (V_pxl.row(x*col + y) + V_pxl.row((x + 1)*col + y + 1)) / 2;
						mesh.V = V_qr;
						f.resize(B.size(), 3);
						for (int k = 0; k < B.size(); k++)
							f.row(k) << k, B.size(), (k + 1) % B.size();
						assert(B.size()+1 == V_qr.rows()&&f.rows()>0);
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
	return meshes;
}