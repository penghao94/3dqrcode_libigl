#include "qrcodeArea.h"
#include <deque>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/foreach.hpp>
#include <omp.h>
#include <igl/triangle/triangulate.h>
#include "igl/ray_mesh_intersect.h"
#include <igl/Hit.h>
double qrcode::qrcodeArea(Engine * engine, Eigen::VectorXd& v_src, Eigen::MatrixXd & V_qr, Eigen::MatrixXd & V_module, Eigen::MatrixXd & Vs, std::vector<Eigen::MatrixXi> & Fs, Eigen::MatrixXd &V, std::vector<Eigen::MatrixXi>& S, Eigen::MatrixXd &rot)
{
	using namespace std;
	#define PI 3.1415926535898
	// Spherical coordinate system
	namespace bg = boost::geometry;
	typedef boost::geometry::model::d2::point_xy<double> Point_c;
	typedef	bg::model::point<double, 2, bg::cs::spherical_equatorial<bg::radian>> Point_s;
	typedef bg::model::polygon<Point_c> Polygon_c;
	typedef bg::model::polygon<Point_s> Polygon_s;
	std::deque<Polygon_c> output;
	Polygon_c qrPolygon, mdPolygon,polygon;
	//origin boundary
	V_qr = (rot*V_qr.transpose()).transpose();
	V_qr.conservativeResize(V_qr.rows() + 1, 3);
	V_qr.row(V_qr.rows() - 1) << V_qr.row(0);
	V_module = (rot*V_module.transpose()).transpose();
	V_module.conservativeResize(V_module.rows() + 1, 3);
	V_module.row(V_module.rows() - 1) << V_module.row(0);
	#pragma omp parallel for schedule(dynamic)
	//spherical qrcode boundary
	for (int i = 0; i < V_qr.rows(); i++) {
		double x = V_qr(i, 0);
		double y = V_qr(i, 1);
		double z = V_qr(i, 2);
		double w = sqrt(pow(x, 2) + pow(y, 2));

		if (y >= 0) {
			if (x >= 0)
				qrPolygon.outer().push_back(Point_c(asin(y / w), asin(z)));
			else
				qrPolygon.outer().push_back(Point_c(PI - asin(y / w), asin(z)));
		}
		else {
			if (x >= 0)
				qrPolygon.outer().push_back(Point_c(asin(y / w), asin(z)));
			else
				qrPolygon.outer().push_back(Point_c(- PI - asin(y / w), asin(z)));
		}
	}
	for (int i = 1; i < qrPolygon.outer().size(); i++) {
		if (qrPolygon.outer()[i-1].x()<0&&qrPolygon.outer()[i].x()>0)
		{
			//y=(y2-y1)/(2pi-x2+x1)*(pi+x1)+y1
			double b = (qrPolygon.outer()[i].y() - qrPolygon.outer()[i-1].y()) / (2 * PI  - qrPolygon.outer()[i].x()+ qrPolygon.outer()[i-1].x())*(PI + qrPolygon.outer()[i - 1].x()) + qrPolygon.outer()[i - 1].y();
			qrPolygon.outer().insert(qrPolygon.outer().begin() + i, Point_c(-PI, b));
			qrPolygon.outer().insert(qrPolygon.outer().begin() + i, Point_c(-PI, 1.57079632));
			qrPolygon.outer().insert(qrPolygon.outer().begin() + i, Point_c(PI, 1.57079632));
			qrPolygon.outer().insert(qrPolygon.outer().begin() + i, Point_c(PI, b));
			break;
		}	
}

	#pragma omp parallel for schedule(dynamic)
	//spherical module boundary
	for (int i = 0; i < V_module.rows(); i++) {
		double x = V_module(i, 0);
		double y = V_module(i, 1);
		double z = V_module(i, 2);
		double w = sqrt(pow(x, 2) + pow(y, 2));

		if (y >= 0) {
			if (x >= 0)
				mdPolygon.outer().push_back(Point_c(asin(y / w), asin(z)));
			else
				mdPolygon.outer().push_back(Point_c(PI - asin(y / w), asin(z)));
		}
		else {
			if (x >= 0)
				mdPolygon.outer().push_back(Point_c(asin(y / w), asin(z)));
			else
				mdPolygon.outer().push_back(Point_c(-PI - asin(y / w), asin(z)));
		}
	}
	for (int i = 1; i < mdPolygon.outer().size(); i++) {
		if (mdPolygon.outer()[i - 1].x() < 0 && mdPolygon.outer()[i].x() > 0)
		{
			//y=(y2-y1)/(2pi-x2+x1)*(pi+x1)+y1
			double b = (mdPolygon.outer()[i].y() - mdPolygon.outer()[i - 1].y()) / (2 * PI - mdPolygon.outer()[i].x() + mdPolygon.outer()[i - 1].x())*(PI + mdPolygon.outer()[i - 1].x()) + mdPolygon.outer()[i - 1].y();
			mdPolygon.outer().insert(mdPolygon.outer().begin() + i, Point_c(-PI, b));
			mdPolygon.outer().insert(mdPolygon.outer().begin() + i, Point_c(-PI, 1.57079632));
			mdPolygon.outer().insert(mdPolygon.outer().begin() + i, Point_c(PI, 1.57079632));
			mdPolygon.outer().insert(mdPolygon.outer().begin() + i, Point_c(PI, b));
			break;
		}
	}

	//intersection
	polygon = qrPolygon;
	bg::intersection(qrPolygon, mdPolygon, output);
	if (output.size() != 0) {
		polygon.clear();
		polygon = output[0];
	}
	Polygon_s s;
	Eigen::MatrixXd E(polygon.outer().size() - 3, 2);
	Eigen::MatrixXd C(polygon.outer().size(), 2);
	
	int index = 0;
	for (int i = 0; i < polygon.outer().size(); i++) {
		s.outer().push_back(Point_s(polygon.outer()[i].x(), polygon.outer()[i].y()));
		C.row(i) << polygon.outer()[i].x(), polygon.outer()[i].y();
		if (polygon.outer()[i].y() != 1.57079632&&i < polygon.outer().size() - 1) {
			E.row(index) << polygon.outer()[i].x(), polygon.outer()[i].y();
			index++;
		}
			
	}
	//random operator
	Eigen::MatrixXd Vd, Seed;
	Eigen::MatrixXi Fd;
	double area = bg::area(s);
	cout << area << endl;
	int num = round(10 * log10(area / 0.00005));
	igl::matlab::mleval(&engine, "clc,clear");
	igl::matlab::mlsetmatrix(&engine, "E", E);
	igl::matlab::mlsetmatrix(&engine, "C", C);
	igl::matlab::mlsetscalar(&engine, "num", num);
	igl::matlab::mlsetscalar(&engine, "PI", PI);
	igl::matlab::mlsetscalar(&engine, "c", C.rows());
	igl::matlab::mleval(&engine, "ave=mean(C(:,2))");
	igl::matlab::mleval(&engine, "Xi=PI.*(2.*rand(1,num)-1)");
	igl::matlab::mleval(&engine, "Yi=interp1(E(:,1),E(:,2),Xi,'linear')");
	igl::matlab::mleval(&engine, "N=floor(num.*(PI/2-Yi)/(PI/2-ave))");
	igl::matlab::mleval(&engine, "seed=[]");
	igl::matlab::mleval(&engine, "for i=1:num;Y=asin((1-sin(Yi(i)))*rand(N(i),1)+sin(Yi(i)));X=repmat(Xi(i),N(i),1);seed=[seed;[X Y]];end");
	igl::matlab::mleval(&engine, "P=[seed;C]");
	igl::matlab::mleval(&engine, "n=sum(N)");
	igl::matlab::mleval(&engine, "con=[(n+1):(n+c);(n+2):(n+c),n+1]");
	igl::matlab::mleval(&engine, "DT = delaunayTriangulation(P,con')");
	igl::matlab::mleval(&engine, "Seed=[cos(seed(:,2)).*cos(seed(:,1)),cos(seed(:,2)).*sin(seed(:,1)),sin(seed(:,2))]");
	igl::matlab::mlgetmatrix(&engine, "DT.Points", Vd);
	igl::matlab::mlgetmatrix(&engine, "DT.connectivityList", Fd);
	igl::matlab::mlgetmatrix(&engine, "Seed", Seed);
	//ray intersect
	Eigen::MatrixXd roti;
	Eigen::VectorXi flag;
	flag.setZero(Seed.rows());
	int count = 0;
	int total = Seed.rows();
	
	roti = rot.inverse().eval();
	Seed = (roti*Seed.transpose()).transpose();
	for (int i = 0; i <Seed.rows(); i++) {
		
		for (int j = 0; j < Fs.size(); j++) {
			std::vector<igl::Hit>hits;
			Eigen::Vector3f src;
			src << v_src(0), v_src(1), v_src(2);
			Eigen::Vector3f dir;
			dir <<Seed(i,0),Seed(i,1),Seed(i,2);
			if (igl::ray_mesh_intersect(src, dir, Vs, Fs[j], hits)) {
				hits.clear();
				if (igl::ray_mesh_intersect(src, dir, V, S[j], hits)) {
					Eigen::Vector3d v1, v2,v3;
					v1 << V(S[j](hits.front().id, 0), 0), V(S[j](hits.front().id, 0), 1), V(S[j](hits.front().id, 0), 2);
					v2 << V(S[j](hits.front().id, 1), 0), V(S[j](hits.front().id, 1), 1), V(S[j](hits.front().id, 1), 2);
					v3 << V(S[j](hits.front().id, 2), 0), V(S[j](hits.front().id, 2), 1), V(S[j](hits.front().id, 2), 2);
					v2 = v2 - v1;
					v3 = v3 - v1;
					v2 = v2.cross(v3);
					v1 = v1 - src.cast<double>();
					if (v1.dot(v2) > 0.01) {
						hits.clear();
						count++;
						cout << "hhh" << endl;
						break;
					}
				}
			}
			
		}
		cout << i << endl;
	}
	flag.resize(0);
	V_qr.resize(0, 0);
	V_module.resize(0, 0);
	E.resize(0, 0);
	C.resize(0, 0);
	Seed.resize(0, 0);
	Vd.resize(0, 0);
	Fd.resize(0, 0);
	
	return area*(1-count/total);  
}
