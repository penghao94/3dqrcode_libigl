#include "visibility.h"
#include "ray_intersect.h"
#include "igl/Hit.h"
#include <Eigen/core>
#include <igl/per_face_normals.h>
bool qrcode::visibility(Engine * engine, Eigen::MatrixXf & Src, Eigen::MatrixXf & Dir, Eigen::MatrixXd & th, Eigen::MatrixXd & th_crv, Eigen::MatrixXd & BW, std::vector<Eigen::MatrixXd>& B_cnn,
	Eigen::MatrixXd & V_fin, Eigen::MatrixXi & F_fin, Eigen::VectorXi & ufct, int v_num, int f_num, Eigen::MatrixXd & vis)
{
	using namespace std;
	using namespace Eigen;
	//scale of a module
	int scale = th.rows() / BW.rows();
	//radius and area of hemisphere
	const double PI = 3.1415926;
	double r = (V_fin.colwise().maxCoeff() - V_fin.colwise().minCoeff()).norm() * 2;
	double A = 2 * PI*r*r;
	//the ratio of visible area and hemisphere
	double ratio;
	Vector3d temp1, temp2;
	//the upper facet
	int f = 0;
	MatrixXi F(f_num, 3);
	MatrixXd N(f_num, 3);
	for (int i = 0; i < F_fin.rows(); i++) {
		//cout << i << "					" << F_fin.row(i) << endl;
		if (ufct(i) == 1) {
			F.row(f) << F_fin.row(i);
			f++;
		}
	}
	igl::per_face_normals(V_fin, F, N);
	//Source and destination for each pixel
	VectorXd v_src(3), v_des(3), v_cent(3), dir(3);
	//UV parameter
	VectorXd uv(2);
	VectorXd uv0(2), uv1(2), uv2(2), e1(2), e2(2);
	double c1, c2, c3, norm;
	uv0 << 0, 0;
	uv1 << 0, 1;
	uv2 << 1, 0;
	//some index pointer
	int blk = 0;
	int bound;
	int size;
	//calculate visibility for each pixel
	for (int i = 0; i < BW.rows() - 1; i++) {
		for (int j = 0; j < BW.cols() - 1; j++) {
			if (BW(i, j) > 0&&i==3&&j==20) {
				//cout << "Black block:" << blk << endl;
				blk++;
				//for each black block
				for (int m = 0; m < 1; m++) {
					for (int n = 0; n < 1; n++) {
						//for each pixel
						int x = i*scale + m;
						int y = j*scale + n;
						cout << "X: " << x << " " << "Y: " << y << endl;

						v_src << ((Src.row(x*th.cols() + y).cast<double>() + Dir.row(x*th.cols() + y).cast<double>()*(th(x, y) + th_crv(x, y)) +
							Src.row((x + 1)*th.cols() + y + 1).cast<double>() + Dir.row((x + 1)*th.cols() + y + 1).cast<double>()*(th(x + 1, y + 1) + th_crv(x + 1, y + 1))
							) / 2).transpose();
						cout << "Src:" << ((Src.row(x*th.cols() + y).cast<double>() + Dir.row(x*th.cols() + y).cast<double>()*(th(x, y)))).transpose() << endl;
						dir << -((Dir.row(x*th.cols() + y) + Dir.row((x + 1)*th.cols() + y + 1)) / 2).transpose().cast<double>();
						dir.normalize();
						cout << dir.transpose() << endl;
						v_cent << v_src + r*dir;
						//	cout << "V_CENT:" << v_cent.transpose() << endl;
						//for each bound pixel
						cout << BW(i, j) - 1 << endl;
						bound = 0;
						ratio = 0;
						MatrixXd V_bound(B_cnn[BW(i, j) - 1].rows(), 3);

						for (int k = 0; k < B_cnn[BW(i, j) - 1].rows(); k++) {
							//	cout << "Bound: " << k << endl;
							dir << B_cnn[BW(i, j) - 1].row(k).transpose() - v_src;
							dir.normalize();
							vector<igl::Hit>hits;
							qrcode::ray_mesh_intersect(v_src, dir, V_fin, F, hits);

							//for each hit
							size = 0;
							for (int l = 0; l < hits.size(); l++) {
								uv << double(hits[l].u), double(hits[l].v);
								e1 = uv - uv0;
								e2 = uv1 - uv0;
								c1 = e1(0)*e2(1) - e1(1)*e2(0);
								e1 = uv - uv1;
								e2 = uv2 - uv1;
								c2 = e1(0)*e2(1) - e1(1)*e2(0);
								e1 = uv - uv2;
								e2 = uv0 - uv2;
								c3 = e1(0)*e2(1) - e1(1)*e2(0);
								norm = dir.dot(N.row(hits[l].id).transpose());
								if (abs(c1) > 0.0000005&&abs(c2) > 0.00000005&&abs(c3) > 0.05&&norm < 0.0000005) {
									size++;
									//cout << "UV:    " << uv.transpose() << endl;
								}

							}
							//cout << "Size:" << size << endl;
							//cout << endl;

							if (size == 0) {
								V_bound.row(bound) << (v_src + r*dir).transpose();
								//cout << "V_BOUND:" << (v_src + r*dir).transpose() << endl;
								bound++;
							}
						}
						cout << endl;
						//cout << "B:" << bound << endl;
						for (int k = 0; k < bound; k++) {
							temp1 << V_bound.row(k%bound).transpose() - v_cent;
							temp2 << v_cent - V_bound.row((k + 1) % bound).transpose();
							ratio += (temp1.cross(temp2)).norm() / 2 / A;
							//cout << "RATIO:" << (temp1.cross(temp2)).norm() / 2 / A << endl;
						}
						cout << "RATIO:" << ratio << endl;
					}
				}
				cout << endl;
			}

		}
	}





	return true;
}