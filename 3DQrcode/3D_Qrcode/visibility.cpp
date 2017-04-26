#include "visibility.h"
#include "outBound.h"
#include "ray_intersect.h"
#include "sphericalPolygen.h"
#include "2D_visible.h"
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
	vis.setZero(BW.rows()*scale, BW.cols()*scale);
	//calculate visibility for each pixel
	for (int i = 0; i < BW.rows() - 1; i++) {
		for (int j = 0; j < BW.cols() - 1; j++) {
			if (BW(i, j) > 0) {
				//cout << "Black block:" << blk << endl;
				blk++;
				//for each black block
				for (int m = 0; m < scale; m++) {
					for (int n = 0; n < scale; n++) {
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
						vis(x, y) = ratio;
					}
				}
				cout << endl;
			}

		}
	}

	return true;
}

bool qrcode::visibility(Engine * engine, Eigen::MatrixXd &V_pxl, Eigen::MatrixXf & Src, Eigen::MatrixXf & Dir, Eigen::MatrixXd & th, Eigen::MatrixXd & th_crv, Eigen::MatrixXd & BW,
	std::vector<Eigen::MatrixXi>& B_cnn, Eigen::MatrixXd & V_fin, Eigen::MatrixXi & F_fin, Eigen::VectorXi & ufct, int v_num, int f_num, Eigen::MatrixXd & vis)
{
	using namespace std;
	using namespace Eigen;
	//scale of a module
	int scale = th.rows() / BW.rows();
	int col = th.cols();
	//radius and area of hemisphere
	const double PI = 3.1415926;
	double r = (V_fin.colwise().maxCoeff() - V_fin.colwise().minCoeff()).norm() * 2;
	double A = 2 * PI*r*r;
	//the ratio of visible area and hemisphere
	double ratio;
	Vector3d edge1, edge2;
	vis.setZero(th.rows(), th.cols());
	//the upper facet
	int f = 0;
	MatrixXi F(f_num, 3);
	MatrixXd N(f_num, 3);
	for (int i = 0; i < F_fin.rows(); i++) {
		if (ufct(i) == 1) {
			F.row(f) << F_fin.row(i);
			f++;
		}
	}
	igl::per_face_normals(V_fin, F, N);
	//Source and destination for each pixel
	VectorXd v_src(3),v_des(3), v_cent(3), dir(3);
	//Bound verticals of 2D light region for each pixel
	vector<Vector2d> B;
	MatrixXd V_bound;
	MatrixXi Bound,temp;
	int bwx = 0;

	//cout << B_cnn[1] << endl;
	/*temp = B_cnn[0];
	for (int i = 0; i < B_cnn[0].rows(); i++) {
		int a = int(temp(i, 0) % BW.cols());
		int b = -int(temp(i, 0) / BW.cols());
		int c = int(temp(i, 1) % BW.cols());
		int d=  -int(temp(i, 1) / BW.cols());
		cout << a<<" "<<b << endl;
		igl::matlab::mlsetscalar(&engine, "a", a);
		igl::matlab::mlsetscalar(&engine, "b", b);
		igl::matlab::mlsetscalar(&engine, "c", c);
		igl::matlab::mlsetscalar(&engine, "d", d);
		igl::matlab::mleval(&engine, "x=[a,b;c,d];");
		igl::matlab::mleval(&engine, "plot(x(:,1),x(:,2));");
		igl::matlab::mleval(&engine, "hold on");
	}*/
	
	//Calculate visibility for each 2D visibility vertex
	for (int i = 0; i < BW.rows() - 1; i++) {
		for (int j = 0; j < BW.cols() - 1; j++) {
			//for each black block
			if (BW(i, j) >0) {
				for (int m = 0; m < scale; m++) {
					for (int n = 0; n < scale; n++) {
						//for each pixel
						int x = i*scale + m;
						int y = j*scale + n;
						//cout << x << ":" << y <<":"<<BW(i,j)<< endl;
						//Shift 0.5 pixel at X and Y axis to centric of each pixel
						v_src << ((Src.row(x*th.cols() + y).cast<double>() + Dir.row(x*th.cols() + y).cast<double>()*(th(x, y) + th_crv(x, y)) +
							Src.row((x + 1)*th.cols() + y + 1).cast<double>() + Dir.row((x + 1)*th.cols() + y + 1).cast<double>()*(th(x + 1, y + 1) + th_crv(x + 1, y + 1))
							) / 2).transpose();
						dir << -((Dir.row(x*th.cols() + y) + Dir.row((x + 1)*th.cols() + y + 1)) / 2).transpose().cast<double>();
						dir.normalize();
						v_cent << v_src + r*dir;
						//To get bound verticals of 2D light region for each pixel

						/*qrcode::outBound(B_cnn[BW(i, j) - 1], 0, scale, Bound);
						MatrixXd p(1, 2);
						p << y + 0.5, -(x+0.5);
						igl::matlab::mlsetmatrix(&engine, "p", p);
						igl::matlab::mleval(&engine, "scatter(p(1),p(2));");
						igl::matlab::mleval(&engine, "hold on");*/
						qrcode::lightRegion(Eigen::RowVector2d(x + 0.5, y + 0.5),B_cnn[BW(i,j)-1], scale, BW.cols(), B);
						MatrixXd V_bound(B.size(), 3);
						for (int k = 0; k < B.size(); k++) {
							int x1 = floor(B[k](1));
							int y1 = floor(B[k](0));
							int x2 = x1 + 1;
							int y2 = y1 + 1;
							v_des = (V_pxl.row(x1*col + y1) + (B[k](1) - floor(B[k](1)))*(V_pxl.row((x1 + 1)*col + y1) - V_pxl.row(x1*col + y1)) + (B[k](0) - floor(B[k](0)))*(V_pxl.row(x1*col + y1 + 1) - V_pxl.row(x1*col + y1))).transpose();
							dir << v_des-v_src;
							dir.normalize();
							V_bound.row(k) << (v_src + r*dir).transpose();
						}

						/*//cout << V_bound << endl;
						igl::matlab::mlsetmatrix(&engine, "temp", V_bound);
						igl::matlab::mlsetscalar(&engine, "i", bwx);
						igl::matlab::mleval(&engine, "figure(i)");
						igl::matlab::mleval(&engine, "plot(temp(:,1),temp(:,2));");		
						//cout  << endl;*/

						//To calculate the ratio of visibility
						ratio = 0;
						for (int k = 0; k < B.size(); k++) {
							edge1 << V_bound.row(k%V_bound.rows()).transpose() - v_cent;
							edge2 << v_cent-V_bound.row((k + 1) % V_bound.rows()).transpose();
							ratio += (edge1.cross(edge2)).norm() / 2 / A;
						}
						V_bound.resize(0, 0);
						if (ratio < 0.001 || ratio>1)
							cout << BW(i, j) << "	" << ratio << endl;
						else
							vis(x, y) = ratio;
							
					}
				}
			}
			bwx++;
		}
	}
	return true;
}

bool qrcode::visibility(Engine *engine, Eigen::MatrixXd &V_pxl, Eigen::MatrixXf &Src, Eigen::MatrixXf &Dir, Eigen::MatrixXd &th, Eigen::MatrixXd &th_crv, Eigen::MatrixXd &BW, std::vector<Eigen::MatrixXi> &B_qr,
	Eigen::Matrix4f &mode, double minZ, double t, Eigen::MatrixXd &box, std::vector<std::vector<Eigen::MatrixXd>>&B_md, Eigen::MatrixXd &V_fin, Eigen::MatrixXd &vis)
{
	using namespace std;
	using namespace Eigen;
	igl::matlab::MatlabWorkspace mw;
	MatrixXd Exp;
	Exp.resize(BW.rows()-1, BW.cols()-1);
	Exp.setConstant(0.5);
	//scale of a module
	int scale = th.rows() / BW.rows();
	int col = th.cols();

	//radius and area of hemisphere
	const double PI = 3.1415926;
	double r = (V_fin.colwise().maxCoeff() - V_fin.colwise().minCoeff()).norm() * 2;
	double A = 2 * PI*r*r;

	//the ratio of visible area and hemisphere
	double ratio;
	Vector3d edge1, edge2;
	vis.setZero(th.rows(), th.cols());
	//Source and destination for each pixel
	VectorXd v_src(3), v_des(3), v_cent(3), dir(3),des(3);
	//Bound verticals of 2D light region for each pixel
	vector<Vector2d> B;
	MatrixXd V_qr;
	//Bound verticals of 3D light region for each pixel
	vector<vector<MatrixXd>> V_md;
	vector<MatrixXd> V_mdl;
	vector<MatrixXd> V_flag;
	//compute translate matrix
	Eigen::VectorXd axis(3);
	axis << 0, 0, 1;
	MatrixXd rot;
	//compute the multi intersect region
	MatrixXd Region;
	//Calculate visibility for each 2D visibility vertex
	for (int i = 0; i < BW.rows() - 1; i++) {
		for (int j = 0; j < BW.cols() - 1; j++) {
			cout<<i<<":"<<j<<endl;
			//for each black block
			if (BW(i, j) >0) {
				for (int m = 0; m < scale; m++) {
					for (int n = 0; n < scale; n++) {
						//for each pixel
						int x = i*scale + m;
						int y = j*scale + n;
						//Shift 0.5 pixel at X and Y axis to centric of each pixel
						v_src << ((Src.row(x*th.cols() + y).cast<double>() + Dir.row(x*th.cols() + y).cast<double>()*(th(x, y) + th_crv(x, y)) +
							Src.row((x + 1)*th.cols() + y + 1).cast<double>() + Dir.row((x + 1)*th.cols() + y + 1).cast<double>()*(th(x + 1, y + 1) + th_crv(x + 1, y + 1))
							) / 2).transpose();
						dir << -((Dir.row(x*th.cols() + y) + Dir.row((x + 1)*th.cols() + y + 1)) / 2).transpose().cast<double>();
						dir.normalize();
						des << (V_pxl.row(x*th.cols() + y).transpose() + V_pxl.row((x + 1)*th.cols() + y + 1).transpose()) / 2;
						v_cent << v_src + r*dir;
						qrcode::rotMatrix(dir, axis, rot);
						/*igl::matlab::mlsetmatrix(&engine, "src", SRC);
						igl::matlab::mleval(&engine, "scatter(src(0),src(1),src(2))");
						igl::matlab::mleval(&engine, "hold on");*/
						//To get qrcode bound verticals of 2D light region for each pixel
						qrcode::lightRegion(Eigen::RowVector2d(x + 0.5, y + 0.5), B_qr[BW(i, j) - 1], scale, BW.cols(), B);
						MatrixXd V_qr(B.size(), 3);
						for (int k = 0; k < B.size(); k++) {
							int x1 = floor(B[k](1));
							int y1 = floor(B[k](0));
							int x2 = x1 + 1;
							int y2 = y1 + 1;
							v_des = (V_pxl.row(x1*col + y1) + (B[k](1) - floor(B[k](1)))*(V_pxl.row((x1 + 1)*col + y1) - V_pxl.row(x1*col + y1)) + (B[k](0) - floor(B[k](0)))*(V_pxl.row(x1*col + y1 + 1) - V_pxl.row(x1*col + y1))).transpose();
							dir << v_des - v_src;
							dir.normalize();
							//V_qr.row(k) << (v_src + r*dir).transpose();
							V_qr.row(k) << dir.transpose();
						}
						//cout << V_qr << endl << endl;
						//To get model bound verticals of 3D light region for each pixel
							/*qrcode::lightRegion(engine,v_src,des, mode,minZ,t,box,B_md,V_md);
							for (int k = 0; k < V_md.size(); k++) {
								V_mdl.push_back((V_md[k][0]-(v_src.transpose().replicate(V_md[k][0].rows(),1))).rowwise().normalized());
								V_flag.push_back(V_md[k][1]);
								//cout << V_mdl[k] << endl;
							}*/
							//double area=qrcode::multiIntersection(engine,V_qr, V_mdl,V_flag,rot, Region)/(4*PI);
							//cout << area << endl;
							double area = qrcode::qrArea(engine, V_qr, rot) / (4 * PI);
							Exp(i,j) = area;
							cout << area << endl;
						/*//To calculate the ratio of visibility
						ratio = 0;
						for (int k = 0; k < B.size(); k++) {
							edge1 << V_bound.row(k%V_bound.rows()).transpose() - v_cent;
							edge2 << v_cent - V_bound.row((k + 1) % V_bound.rows()).transpose();
							ratio += (edge1.cross(edge2)).norm() / 2 / A;
						}
						V_bound.resize(0, 0);
						if (ratio < 0.001 || ratio>1)
							cout << BW(i, j) << "	" << ratio << endl;
						else
							vis(x, y) = ratio;*/

					}
				}
			}
		}
	}
	mw.save(Exp, "Exp");
	mw.write("Experiment.mat");
	V_mdl.clear();
	return true;
}
