#include "visibility.h"
#include "outBound.h"
#include "ray_intersect.h"
#include "sphericalPolygen.h"
#include "2D_visible.h"
#include "igl/Hit.h"
#include <Eigen/core>
#include <igl/per_face_normals.h>
#include "visPolygon.h"
#include <omp.h>
#include "qrcodeArea.h"
#include "ambient_occlusion.h"
bool qrcode::visibility(Engine *engine, Eigen::MatrixXd &V_pxl, Eigen::MatrixXf &Src, Eigen::MatrixXf &Dir, Eigen::MatrixXd &th, Eigen::MatrixXd &th_crv, Eigen::MatrixXd &BW, std::vector<Eigen::MatrixXi> &B_qr,
	Eigen::Matrix4f &mode, double minZ, double t, Eigen::MatrixXd &box, std::vector<std::vector<Eigen::MatrixXd>>&B_md, Eigen::MatrixXd &vis)
{
	using namespace std;
	using namespace Eigen;
	
	//scale of a module
	int scale = th.rows() / BW.rows();
	int col = th.cols();

	//radius and area of hemisphere
	const double PI = 3.1415926;

	//the ratio of visible area and hemisphere
	double ratio;
	vis.resize((BW.rows() - 1)*scale, (BW.cols() - 1)*scale);
	vis.setConstant(0.5);
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
						v_src << ((Src.row(x*th.cols() + y).cast<double>() + Dir.row(x*th.cols() + y).cast<double>()*(th(x, y) + th_crv(i, j)) +
							Src.row((x + 1)*th.cols() + y + 1).cast<double>() + Dir.row((x + 1)*th.cols() + y + 1).cast<double>()*(th(x + 1, y + 1) + th_crv(i, j))
							) / 2).transpose();
						dir << -((Dir.row(x*th.cols() + y) + Dir.row((x + 1)*th.cols() + y + 1)) / 2).transpose().cast<double>();
						dir.normalize();
						des << (V_pxl.row(x*th.cols() + y).transpose() + V_pxl.row((x + 1)*th.cols() + y + 1).transpose()) / 2;
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
							qrcode::lightRegion(engine,v_src,des, mode,minZ,t,box,B_md,V_md);
							for (int k = 0; k < V_md.size(); k++) {
								V_mdl.push_back((V_md[k][0]-(v_src.transpose().replicate(V_md[k][0].rows(),1))).rowwise().normalized());
								V_flag.push_back(V_md[k][1]);
								//cout << V_mdl[k] << endl;
							}
							//cout << V_mdl.size() << endl;
							double area=qrcode::multiIntersection(engine,V_qr, V_mdl,V_flag,rot, Region)/(4*PI);
							//cout << area << endl;
							/*double area = qrcode::qrArea(engine, V_qr, rot) / (4 * PI);*/
							vis(x,y) = area;
							cout << area << endl;
							V_mdl.clear();
							V_flag.clear();
						
					}
				}
			}
			else if ( BW(i, j) ==0) {

				for (int m = 0; m < scale; m++) {
					for (int n = 0; n < scale; n++) {
						//for each pixel
						int x = i*scale + m;
						int y = j*scale + n;
						//Shift 0.5 pixel at X and Y axis to centric of each pixel
						v_src << (V_pxl.row(x*th.cols() + y).transpose() + V_pxl.row((x + 1)*th.cols() + y + 1).transpose()) / 2;
						dir << -((Dir.row(x*th.cols() + y) + Dir.row((x + 1)*th.cols() + y + 1)) / 2).transpose().cast<double>();
						dir.normalize();
						des << (Src.row(x*th.cols() + y).transpose() + Src.row((x + 1)*th.cols() + y + 1).transpose()).cast<double>() / 2;
						qrcode::rotMatrix(dir, axis, rot);
						//to get qrcode bound verticals
						MatrixXd V_qr1(4, 3);
						V_qr1.row(0) << V_pxl.row(i*scale*th.cols() + j*scale);
						V_qr1.row(1) << V_pxl.row(i*scale*th.cols() + (j+1)*scale);
						V_qr1.row(2) << V_pxl.row((i+1)*scale*th.cols() + (j + 1)*scale);
						V_qr1.row(3) << V_pxl.row((i+1)*scale*th.cols() + j*scale);
						V_qr1 = (V_qr1 - v_src.transpose().replicate(4, 1)).rowwise().normalized();
						//cout << V_qr1 << endl;
						//To get model bound verticals of 3D light region for each pixel
						qrcode::lightRegion(engine, v_src, des, mode, minZ, t, box, B_md, V_md);
						for (int k = 0; k < V_md.size(); k++) {
							V_mdl.push_back((V_md[k][0] - (v_src.transpose().replicate(V_md[k][0].rows(), 1))).rowwise().normalized());
							V_flag.push_back(V_md[k][1]);
						//	cout << V_md[k][0] << endl;
						}
						//cout << V_mdl.size() << endl;
						double area = qrcode::multiIntersection(engine,V_qr1,V_mdl, V_flag, rot, Region) / (4 * PI);
						//cout << area << endl;
						/*double area = qrcode::qrArea(engine, V_qr, rot) / (4 * PI);*/
						if(area<0.5&&area>0.1)
							vis(x, y) = area;

						cout << area << endl;
						V_mdl.clear();
						V_flag.clear();

					}
				}
			}
		}
	}
	V_mdl.clear();
	return true;
}
bool qrcode::visibility_t(Engine *engine, Eigen::MatrixXd &V_pxl, Eigen::MatrixXf &Src, Eigen::MatrixXf &Dir, Eigen::MatrixXd &th, Eigen::MatrixXd &th_crv, Eigen::MatrixXd &BW, std::vector<Eigen::MatrixXi> &B_qr,
	Eigen::Matrix4f &mode, double minZ, double t, Eigen::MatrixXd &box, std::vector<std::vector<Eigen::MatrixXd>>&B_md, Eigen::MatrixXd &vis)
{
	using namespace std;
	using namespace Eigen;
	igl::matlab::MatlabWorkspace mw;
	//scale of a module
	int scale = th.rows() / BW.rows();
	int col = th.cols();

	//radius and area of hemisphere
	const double PI = 3.1415926;

	//the ratio of visible area and hemisphere
	double ratio;
	vis.resize((BW.rows() - 1)*scale, (BW.cols() - 1)*scale);
	vis.setConstant(1);
	//Source and destination for each pixel
	VectorXd v_src(3), v_des(3), v_cent(3), dir(3), des(3);
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
			cout << i << ":" << j << endl;
			//for each black block
			if (BW(i, j) >0) {
				for (int m = 0; m < scale; m++) {
					for (int n = 0; n < scale; n++) {
						//for each pixel
						int x = i*scale + m;
						int y = j*scale + n;
						//Shift 0.5 pixel at X and Y axis to centric of each pixel
						v_src << ((Src.row(x*th.cols() + y).cast<double>() + Dir.row(x*th.cols() + y).cast<double>()*(th(x, y) + th_crv(i, j)) +
							Src.row((x + 1)*th.cols() + y + 1).cast<double>() + Dir.row((x + 1)*th.cols() + y + 1).cast<double>()*(th(x + 1, y + 1) + th_crv(i, j))
							) / 2).transpose();
						dir << -((Dir.row(x*th.cols() + y) + Dir.row((x + 1)*th.cols() + y + 1)) / 2).transpose().cast<double>();
						dir.normalize();
						des << (V_pxl.row(x*th.cols() + y).transpose() + V_pxl.row((x + 1)*th.cols() + y + 1).transpose()) / 2;
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
						double area = qrcode::qrArea(engine, V_qr, rot) / (4 * PI);
						vis(x, y) = area;
						cout << area << endl;

					}
				}
			}
		}
	}
	mw.save(vis, "V");
	mw.write("test.mat");
	V_mdl.clear();
	return true;
}
bool qrcode::visibility2(Engine *engine, Eigen::MatrixXd &V_pxl, Eigen::MatrixXf &Src, Eigen::MatrixXf &Dir, Eigen::MatrixXd &th, Eigen::MatrixXd &th_crv, Eigen::MatrixXd &BW, std::vector<Eigen::MatrixXi> &B_qr,
	Eigen::Matrix4f &mode, double minZ, double t, Eigen::MatrixXd &box, std::vector<std::vector<Eigen::MatrixXd>>&B_md, Eigen::MatrixXd &vis)
{
	using namespace std;
	using namespace Eigen;

	//scale of a module
	int scale = th.rows() / BW.rows();
	int col = th.cols();

	//radius and area of hemisphere
	const double PI = 3.1415926;

	//the ratio of visible area and hemisphere
	double ratio;
	//Source and destination for each pixel
	VectorXd v_src(3), v_des(3), v_cent(3), dir(3), des(3);
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
			cout << i << ":" << j << endl;
			//for each black block
			if (BW(i, j) >0) {
				for (int m = 0; m < scale; m++) {
					for (int n = 0; n < scale; n++) {
						//for each pixel
						int x = i*scale + m;
						int y = j*scale + n;
						//Shift 0.5 pixel at X and Y axis to centric of each pixel
						v_src << ((Src.row(x*th.cols() + y).cast<double>() + Dir.row(x*th.cols() + y).cast<double>()*(th(x, y) + th_crv(i, j)) +
							Src.row((x + 1)*th.cols() + y + 1).cast<double>() + Dir.row((x + 1)*th.cols() + y + 1).cast<double>()*(th(x + 1, y + 1) + th_crv(i , j))
							) / 2).transpose();
						dir << -((Dir.row(x*th.cols() + y) + Dir.row((x + 1)*th.cols() + y + 1)) / 2).transpose().cast<double>();
						dir.normalize();
						des << (V_pxl.row(x*th.cols() + y).transpose() + V_pxl.row((x + 1)*th.cols() + y + 1).transpose()) / 2;
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
						qrcode::lightRegion(engine, v_src, des, mode, minZ, t, box, B_md, V_md);
						for (int k = 0; k < V_md.size(); k++) {
							V_mdl.push_back((V_md[k][0] - (v_src.transpose().replicate(V_md[k][0].rows(), 1))).rowwise().normalized());
							V_flag.push_back(V_md[k][1]);
							//cout << V_mdl[k] << endl;
						}
						//to get qrcode bound verticals
						MatrixXd V_qr1(0, 3);
						
						if (th_crv(i,j-1) == th_crv(i-1,j)) {
							V_qr1.conservativeResize(V_qr1.rows() + 1, 3);
							V_qr1.row(V_qr1.rows() - 1) << Src.row(i*scale*th.cols() + j*scale).cast<double>() + Dir.row(i*scale*th.cols() + j*scale).cast<double>()*(th(i*scale, j*scale) + th_crv(i,j-1));
						}
						else {
							V_qr1.conservativeResize(V_qr1.rows() + 2, 3);
							V_qr1.row(V_qr1.rows() - 1) << Src.row(i*scale*th.cols() + j*scale).cast<double>() + Dir.row(i*scale*th.cols() + j*scale).cast<double>()*(th(i*scale, j*scale) + th_crv(i-1, j));
							V_qr1.row(V_qr1.rows() - 2) << Src.row(i*scale*th.cols() + j*scale).cast<double>() + Dir.row(i*scale*th.cols() + j*scale).cast<double>()*(th(i*scale, j*scale) + th_crv(i, j - 1));

						}
						if (th_crv(i, j - 1) == th_crv(i + 1, j)) {
							V_qr1.conservativeResize(V_qr1.rows() + 1, 3);
							V_qr1.row(V_qr1.rows() - 1) << Src.row(((i + 1)*scale)*th.cols() + j*scale).cast<double>() + Dir.row(((i + 1)*scale)*th.cols() + j*scale).cast<double>()*(th((i + 1)*scale, j*scale) + th_crv(i + 1, j));
						}
						else {
							V_qr1.conservativeResize(V_qr1.rows() + 2, 3);
							V_qr1.row(V_qr1.rows() - 1) << Src.row(((i + 1)*scale)*th.cols() + j*scale).cast<double>() + Dir.row(((i + 1)*scale)*th.cols() + j*scale).cast<double>()*(th((i + 1)*scale, j*scale) + th_crv(i, j - 1));
							V_qr1.row(V_qr1.rows() - 1) << Src.row(((i + 1)*scale)*th.cols() + j*scale).cast<double>() + Dir.row(((i + 1)*scale)*th.cols() + j*scale).cast<double>()*(th((i + 1)*scale, j*scale) + th_crv(i + 1, j));

						}
						if (th_crv(i, j + 1) == th_crv(i + 1, j)) {
							V_qr1.conservativeResize(V_qr1.rows() + 1, 3);
							V_qr1.row(V_qr1.rows() - 1) << Src.row(((i + 1)*scale)*th.cols() + (j + 1)*scale).cast<double>() + Dir.row(((i + 1)*scale)*th.cols() + (j + 1)*scale).cast<double>()*(th((i + 1)*scale, (j + 1)*scale) + th_crv(i, j + 1));
						}
						else {
							V_qr1.conservativeResize(V_qr1.rows() + 2, 3);
							V_qr1.row(V_qr1.rows() - 1) << Src.row(((i + 1)*scale)*th.cols() + (j + 1)*scale).cast<double>() + Dir.row(((i + 1)*scale)*th.cols() + (j + 1)*scale).cast<double>()*(th((i + 1)*scale, (j + 1)*scale) + th_crv((i + 1)*scale, j*scale));
							V_qr1.row(V_qr1.rows() - 2) << Src.row(((i + 1)*scale)*th.cols() + (j + 1)*scale).cast<double>() + Dir.row(((i + 1)*scale)*th.cols() + (j + 1)*scale).cast<double>()*(th((i + 1)*scale, (j + 1)*scale) + th_crv(i*scale, (j + 1)*scale));

						}
						if (th_crv(i, j + 1) == th_crv(i - 1, j)) {
							V_qr1.conservativeResize(V_qr1.rows() + 1, 3);
							V_qr1.row(V_qr1.rows() - 1) << Src.row(i*scale*th.cols() + (j + 1)*scale).cast<double>() + Dir.row(i*scale*th.cols() + (j + 1)*scale).cast<double>()*(th(i*scale, (j + 1)*scale) + th_crv(i - 1, j));
						}
						else {
							V_qr1.conservativeResize(V_qr1.rows() + 2, 3);
							V_qr1.row(V_qr1.rows() - 1) << Src.row(i*scale*th.cols() + (j + 1)*scale).cast<double>() + Dir.row(i*scale*th.cols() + (j + 1)*scale).cast<double>()*(th(i*scale, (j + 1)*scale) + th_crv(i, j + 1));
							V_qr1.row(V_qr1.rows() - 2) << Src.row(i*scale*th.cols() + (j + 1)*scale).cast<double>() + Dir.row(i*scale*th.cols() + (j + 1)*scale).cast<double>()*(th(i*scale, (j + 1)*scale) + th_crv(i - 1, j));

						}
						
						
						V_qr1 = (V_qr1 - v_src.transpose().replicate(V_qr1.rows(), 1)).rowwise().normalized();
						V_mdl.push_back(V_qr1);
						Eigen::MatrixXd Flg;
						Flg.setZero(V_qr1.rows(), 1);
						V_flag.push_back(Flg);
						//cout << V_mdl.size() << endl;
						double area = qrcode::multiIntersection(engine, V_qr, V_mdl, V_flag, rot, Region) / (4 * PI);
						//cout << area << endl;
						/*double area = qrcode::qrArea(engine, V_qr, rot) / (4 * PI);*/
						vis(x, y) = area;
						cout << area << endl;
						V_mdl.clear();
						V_flag.clear();

					}
				}
			}
		}
	}
	V_mdl.clear();
	return true;
}

bool qrcode::visibility(Engine * engine, Eigen::MatrixXd & V_pxl, Eigen::MatrixXf & Src, Eigen::MatrixXf & Dir, Eigen::MatrixXd & th, Eigen::MatrixXd & th_crv, Eigen::MatrixXd & BW, std::vector<Eigen::MatrixXi>& B_qr, Eigen::MatrixXd & Vs, std::vector<Eigen::MatrixXi>& Fs, Eigen::MatrixXd & V_fin, std::vector<Eigen::MatrixXi>& S, Eigen::Matrix4f & mode, double minZ, double t, Eigen::MatrixXd & box, std::vector<std::vector<Eigen::MatrixXd>>& B_md, Eigen::MatrixXd & vis)
{
	using namespace std;
	using namespace Eigen;

	//scale of a module
	int scale = th.rows() / BW.rows();
	int col = th.cols();

	//radius and area of hemisphere
	const double PI = 3.1415926;

	//the ratio of visible area and hemisphere
	double ratio;
	//Source and destination for each pixel
	VectorXd v_src(3), v_des(3), v_cent(3), dir(3), des(3);
	//Bound verticals of 2D light region for each pixel
	vector<Vector2d> B;
	MatrixXd V_qr;
	//Bound verticals of 3D light region for each pixel
	//compute translate matrix
	Eigen::VectorXd axis(3);
	axis << 0, 0, 1;
	MatrixXd rot;
	//compute the multi intersect region
	MatrixXd Region;
	//Calculate visibility for each 2D visibility vertex
	for (int i = 0; i < BW.rows() - 1; i++) {
		for (int j = 0; j < BW.cols() - 1; j++) {
			cout << i << ":" << j << endl;
			//for each black block
			if (BW(i, j) >0) {
				for (int m = 0; m < scale; m++) {
					for (int n = 0; n < scale; n++) {
						//for each pixel
						int x = i*scale + m;
						int y = j*scale + n;
						//Shift 0.5 pixel at X and Y axis to centric of each pixel
						v_src << ((Src.row(x*th.cols() + y).cast<double>() + Dir.row(x*th.cols() + y).cast<double>()*(th(x, y) + th_crv(i, j)) +
							Src.row((x + 1)*th.cols() + y + 1).cast<double>() + Dir.row((x + 1)*th.cols() + y + 1).cast<double>()*(th(x + 1, y + 1) + th_crv(i, j))
							) / 2).transpose();
						dir << -((Dir.row(x*th.cols() + y) + Dir.row((x + 1)*th.cols() + y + 1)) / 2).transpose().cast<double>();
						dir.normalize();
						des << (V_pxl.row(x*th.cols() + y).transpose() + V_pxl.row((x + 1)*th.cols() + y + 1).transpose()) / 2;
						qrcode::rotMatrix(dir, axis, rot);
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
						//to get qrcode bound verticals
						MatrixXd V_qr1(0, 3);

						if (th_crv(i, j - 1) == th_crv(i - 1, j)) {
							V_qr1.conservativeResize(V_qr1.rows() + 1, 3);
							V_qr1.row(V_qr1.rows() - 1) << Src.row(i*scale*th.cols() + j*scale).cast<double>() + Dir.row(i*scale*th.cols() + j*scale).cast<double>()*(th(i*scale, j*scale) + th_crv(i, j - 1));
						}
						else {
							V_qr1.conservativeResize(V_qr1.rows() + 2, 3);
							V_qr1.row(V_qr1.rows() - 1) << Src.row(i*scale*th.cols() + j*scale).cast<double>() + Dir.row(i*scale*th.cols() + j*scale).cast<double>()*(th(i*scale, j*scale) + th_crv(i - 1, j));
							V_qr1.row(V_qr1.rows() - 2) << Src.row(i*scale*th.cols() + j*scale).cast<double>() + Dir.row(i*scale*th.cols() + j*scale).cast<double>()*(th(i*scale, j*scale) + th_crv(i, j - 1));

						}
						if (th_crv(i, j - 1) == th_crv(i + 1, j)) {
							V_qr1.conservativeResize(V_qr1.rows() + 1, 3);
							V_qr1.row(V_qr1.rows() - 1) << Src.row(((i + 1)*scale)*th.cols() + j*scale).cast<double>() + Dir.row(((i + 1)*scale)*th.cols() + j*scale).cast<double>()*(th((i + 1)*scale, j*scale) + th_crv(i + 1, j));
						}
						else {
							V_qr1.conservativeResize(V_qr1.rows() + 2, 3);
							V_qr1.row(V_qr1.rows() - 1) << Src.row(((i + 1)*scale)*th.cols() + j*scale).cast<double>() + Dir.row(((i + 1)*scale)*th.cols() + j*scale).cast<double>()*(th((i + 1)*scale, j*scale) + th_crv(i, j - 1));
							V_qr1.row(V_qr1.rows() - 1) << Src.row(((i + 1)*scale)*th.cols() + j*scale).cast<double>() + Dir.row(((i + 1)*scale)*th.cols() + j*scale).cast<double>()*(th((i + 1)*scale, j*scale) + th_crv(i + 1, j));

						}
						if (th_crv(i, j + 1) == th_crv(i + 1, j)) {
							V_qr1.conservativeResize(V_qr1.rows() + 1, 3);
							V_qr1.row(V_qr1.rows() - 1) << Src.row(((i + 1)*scale)*th.cols() + (j + 1)*scale).cast<double>() + Dir.row(((i + 1)*scale)*th.cols() + (j + 1)*scale).cast<double>()*(th((i + 1)*scale, (j + 1)*scale) + th_crv(i, j + 1));
						}
						else {
							V_qr1.conservativeResize(V_qr1.rows() + 2, 3);
							V_qr1.row(V_qr1.rows() - 1) << Src.row(((i + 1)*scale)*th.cols() + (j + 1)*scale).cast<double>() + Dir.row(((i + 1)*scale)*th.cols() + (j + 1)*scale).cast<double>()*(th((i + 1)*scale, (j + 1)*scale) + th_crv((i + 1)*scale, j*scale));
							V_qr1.row(V_qr1.rows() - 2) << Src.row(((i + 1)*scale)*th.cols() + (j + 1)*scale).cast<double>() + Dir.row(((i + 1)*scale)*th.cols() + (j + 1)*scale).cast<double>()*(th((i + 1)*scale, (j + 1)*scale) + th_crv(i*scale, (j + 1)*scale));

						}
						if (th_crv(i, j + 1) == th_crv(i - 1, j)) {
							V_qr1.conservativeResize(V_qr1.rows() + 1, 3);
							V_qr1.row(V_qr1.rows() - 1) << Src.row(i*scale*th.cols() + (j + 1)*scale).cast<double>() + Dir.row(i*scale*th.cols() + (j + 1)*scale).cast<double>()*(th(i*scale, (j + 1)*scale) + th_crv(i - 1, j));
						}
						else {
							V_qr1.conservativeResize(V_qr1.rows() + 2, 3);
							V_qr1.row(V_qr1.rows() - 1) << Src.row(i*scale*th.cols() + (j + 1)*scale).cast<double>() + Dir.row(i*scale*th.cols() + (j + 1)*scale).cast<double>()*(th(i*scale, (j + 1)*scale) + th_crv(i, j + 1));
							V_qr1.row(V_qr1.rows() - 2) << Src.row(i*scale*th.cols() + (j + 1)*scale).cast<double>() + Dir.row(i*scale*th.cols() + (j + 1)*scale).cast<double>()*(th(i*scale, (j + 1)*scale) + th_crv(i - 1, j));

						}


						V_qr1 = (V_qr1 - v_src.transpose().replicate(V_qr1.rows(), 1)).rowwise().normalized();
						double area = qrcode::qrcodeArea(engine, v_src, V_qr, V_qr1, Vs, Fs, V_fin, S, rot);
						vis(x, y) = area;
						cout << area << endl;

					}
				}
			}
		}
	}
	return true;
}

