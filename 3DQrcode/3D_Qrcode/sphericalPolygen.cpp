#include "sphericalPolygen.h"
#include <deque>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/foreach.hpp>

double qrcode::multiIntersection(Engine *engine, Eigen::MatrixXd & V_qr, std::vector<Eigen::MatrixXd>& V_md, std::vector<Eigen::MatrixXd>&V_flag, Eigen::MatrixXd & rot, Eigen::MatrixXd & dir)
{
	
	using namespace std;
	const double pi = 3.1415926535898;
	//Eigen::MatrixXd temp;
	Eigen::VectorXd a(2);
	double b;
	int index;
	// Spherical coordinate system
	namespace bg = boost::geometry;
	typedef boost::geometry::model::d2::point_xy<double> Point_c;
	typedef	bg::model::point<double, 2, bg::cs::spherical_equatorial<bg::radian>> Point_s;
	typedef bg::model::polygon<Point_c> Polygon;
	typedef bg::model::polygon<Point_s> Polygon_s;
	Polygon t,r;
	double x, y, z,w,flag;
	
	std::deque<Polygon> output;
	std::vector<Polygon> polygons;

	polygons.resize(V_md.size() + 1);
	V_qr = (rot*V_qr.transpose()).transpose();
	//temp.resize(V_qr.rows()+5, 3);
	

	/*
	For qrcode boundary
	*/
	index = 0;
	a << 1, 1;
	for (int i = 0; i < V_qr.rows(); i++) {
		x = V_qr(i, 0);
		y = V_qr(i, 1);
		z = V_qr(i, 2);
		w = sqrt(pow(x, 2) + pow(y, 2));
		if (y >= 0) {
			if (x >= 0) {
				polygons[0].outer().push_back(Point_c(asin(y / w), asin(z)));
				a << asin(y / w), asin(z);
			}	
			else{
				if (a(0) < 0) {
					//y=(y2-y1)/(2pi-x2+x1)*(pi+x1)+y1
					b = (asin(z) - a(1)) / (2 * pi - pi + asin(y / w) + a(0))*(pi + a(0)) + a(1);
					polygons[0].outer().push_back(Point_c(-pi, b));
					//temp.row(index) << polygons[0].outer().back().x(), polygons[0].outer().back().y(), 0;
					//index++;
					polygons[0].outer().push_back(Point_c(-pi, 1.57079632));
					//temp.row(index) << polygons[0].outer().back().x(), polygons[0].outer().back().y(), 0;
					//index++;
					polygons[0].outer().push_back(Point_c(pi, 1.57079632));
					//temp.row(index) << polygons[0].outer().back().x(), polygons[0].outer().back().y(), 0;
					//index++;
					polygons[0].outer().push_back(Point_c(pi, b));
					//temp.row(index) << polygons[0].outer().back().x(), polygons[0].outer().back().y(), 0;
					//index++;
					polygons[0].outer().push_back(Point_c(pi - asin(y / w), asin(z)));
				}
				else
					polygons[0].outer().push_back(Point_c(pi - asin(y / w), asin(z)));
				a << pi - asin(y / w), asin(z);
			}
				
		}
		else {
			if (x >= 0) {
				polygons[0].outer().push_back(Point_c(asin(y / w), asin(z)));
				a << asin(y / w), asin(z);
			}
			else {
				polygons[0].outer().push_back(Point_c(-pi - asin(y / w), asin(z)));
				a << -pi - asin(y / w), asin(z);
			}
				
		}
		//temp.row(index) << polygons[0].outer().back().x(), polygons[0].outer().back().y(),0;
		//index++;
	}
	
	x = V_qr(0, 0);
	y = V_qr(0, 1);
	z = V_qr(0, 2);
	w = sqrt(pow(x, 2) + pow(y, 2));
	if (y >= 0) {
		if (x >= 0) {
			polygons[0].outer().push_back(Point_c(asin(y / w), asin(z)));
			a << asin(y / w), asin(z);
		}
		else {
			if (a(0) < 0) {
				//y=(y2-y1)/(2pi-x2+x1)*(pi+x1)+y1
				b = (asin(z) - a(1)) / (2 * pi - pi + asin(y / w) + a(0))*(pi + a(0)) + a(1);
				polygons[0].outer().push_back(Point_c(-pi, b));
				//temp.row(index) << polygons[0].outer().back().x(), polygons[0].outer().back().y(), 0;
				//index++;
				polygons[0].outer().push_back(Point_c(-pi, 1.57079632));
				//temp.row(index) << polygons[0].outer().back().x(), polygons[0].outer().back().y(), 0;
			//	index++;
				polygons[0].outer().push_back(Point_c(pi, 1.57079632));
				//temp.row(index) << polygons[0].outer().back().x(), polygons[0].outer().back().y(), 0;
			//	index++;
				polygons[0].outer().push_back(Point_c(pi, b));
				//temp.row(index) << polygons[0].outer().back().x(), polygons[0].outer().back().y(), 0;
				//index++;
				polygons[0].outer().push_back(Point_c(pi - asin(y / w), asin(z)));
			}
			else
				polygons[0].outer().push_back(Point_c(pi - asin(y / w), asin(z)));
			a << pi - asin(y / w), asin(z);
		}

	}
	else {
		if (x >= 0) {
			polygons[0].outer().push_back(Point_c(asin(y / w), asin(z)));
			a << asin(y / w), asin(z);
		}
		else {
			polygons[0].outer().push_back(Point_c(-pi - asin(y / w), asin(z)));
			a << -pi - asin(y / w), asin(z);
		}

	}
	//temp.row(temp.rows()-1) << polygons[0].outer().back().x(), polygons[0].outer().back().y(),0;

	//igl::matlab::mlsetmatrix(&engine, "temp", temp);
	//igl::matlab::mleval(&engine, "plot(temp(:, 1), temp(:, 2))");
	//igl::matlab::mleval(&engine, "hold on");
	//cout << bg::wkt(polygons[0]) << endl;
	//cout << bg::area(polygons[0]) << endl;


	/*
	For model slice boundary
	*/
	index = 0;
	for (int i = 0; i < V_md.size(); i++) {
		V_md[i] = (rot*V_md[i].transpose()).transpose();
		//cout << V_md[i] << endl;
		//temp.resize(V_md[i].rows() + 1,3);
		index = 0;
		a << 1, 1;
		for (int j = 0; j < V_md[i].rows(); j++) {
			flag = V_flag[i]((V_md[i].rows() - 1 - j, 0));
			x = V_md[i](V_md[i].rows() - 1 - j, 0);
			y = V_md[i](V_md[i].rows() - 1 - j, 1);

			if (flag == 0)
				z = V_md[i](V_md[i].rows() - 1 - j, 2);
			else
				z = -0.9999999;
			w = sqrt(pow(x, 2) + pow(y, 2));
			
			if (y >= 0) {
				if (x >= 0) {
					polygons[i + 1].outer().push_back(Point_c(asin(y / w), asin(z)));
					a << asin(y / w), asin(z);
				}
				else {
					if (a(0) < 0) {
						//y=(y2-y1)/(2pi-x2+x1)*(pi+x1)+y1
					//	temp.conservativeResize(temp.rows() + 4, 3);
						b = (asin(z) - a(1)) / (2 * pi - pi + asin(y / w) + a(0))*(pi + a(0)) + a(1);
						polygons[i + 1].outer().push_back(Point_c(-pi, b));
						//temp.row(index) << polygons[i+1].outer().back().x(), polygons[i + 1].outer().back().y(), 0;
					//	index++;
						polygons[i + 1].outer().push_back(Point_c(-pi, 1.57079632));
					//	temp.row(index) << polygons[i + 1].outer().back().x(), polygons[i + 1].outer().back().y(), 0;
					//	index++;
						polygons[i + 1].outer().push_back(Point_c(pi, 1.57079632));
					//	temp.row(index) << polygons[i + 1].outer().back().x(), polygons[i + 1].outer().back().y(), 0;
					//	index++;
						polygons[i + 1].outer().push_back(Point_c(pi, b));
					//	temp.row(index) << polygons[i + 1].outer().back().x(), polygons[i + 1].outer().back().y(), 0;
					//	index++;
						polygons[i + 1].outer().push_back(Point_c(pi - asin(y / w), asin(z)));
					}
					else
						polygons[i + 1].outer().push_back(Point_c(pi - asin(y / w), asin(z)));
					a << pi - asin(y / w), asin(z);
				}

			}
			else {
				if (x >= 0) {
					polygons[i + 1].outer().push_back(Point_c(asin(y / w), asin(z)));
					a << asin(y / w), asin(z);
				}
				else {
					polygons[i + 1].outer().push_back(Point_c(-pi - asin(y / w), asin(z)));
					a << -pi - asin(y / w), asin(z);
				}

			}
			//temp.row(index) << polygons[i+1].outer().back().x(), polygons[i+1].outer().back().y(), 0;
			index++;
		}
		x = V_md[i](V_md[i].rows()-1, 0);
		y = V_md[i](V_md[i].rows() - 1, 1);
		if (flag == 0)
			z = V_md[i](V_md[i].rows() - 1, 2);
		else
			z = -0.9999999;
		w = sqrt(pow(x, 2) + pow(y, 2));
		if (y >= 0) {
			if (x >= 0) {
				polygons[i + 1].outer().push_back(Point_c(asin(y / w), asin(z)));
				a << asin(y / w), asin(z);
			}
			else {
				if (a(0) < 0) {
					//y=(y2-y1)/(2pi-x2+x1)*(pi+x1)+y1
					//temp.conservativeResize(temp.rows() + 4, 3);
					b = (asin(z) - a(1)) / (2 * pi - pi + asin(y / w) + a(0))*(pi + a(0)) + a(1);
					polygons[i + 1].outer().push_back(Point_c(-pi, b));
					//temp.row(index) << polygons[i + 1].outer().back().x(), polygons[i + 1].outer().back().y(), 0;
				//	index++;
					polygons[i + 1].outer().push_back(Point_c(-pi, 1.57079632));
				//	temp.row(index) << polygons[i + 1].outer().back().x(), polygons[i + 1].outer().back().y(), 0;
				//	index++;
					polygons[i + 1].outer().push_back(Point_c(pi, 1.57079632));
				//	temp.row(index) << polygons[i + 1].outer().back().x(), polygons[i + 1].outer().back().y(), 0;
					//index++;
					polygons[i + 1].outer().push_back(Point_c(pi, b));
					//temp.row(index) << polygons[i + 1].outer().back().x(), polygons[i + 1].outer().back().y(), 0;
					//index++;
					polygons[i + 1].outer().push_back(Point_c(pi - asin(y / w), asin(z)));
				}
				else
					polygons[i + 1].outer().push_back(Point_c(pi - asin(y / w), asin(z)));
				a << pi - asin(y / w), asin(z);
			}

		}
		else {
			if (x >= 0) {
				polygons[i + 1].outer().push_back(Point_c(asin(y / w), asin(z)));
				a << asin(y / w), asin(z);
			}
			else {
				polygons[i + 1].outer().push_back(Point_c(-pi - asin(y / w), asin(z)));
				a << -pi - asin(y / w), asin(z);
			}

		}
		
		//temp.row(temp.rows() - 1) << polygons[i + 1].outer().back().x(), polygons[i + 1].outer().back().y(), 0;
		//igl::matlab::mlsetmatrix(&engine, "temp", temp);
		//igl::matlab::mleval(&engine, "plot(temp(:,1),temp(:,2))");
		//igl::matlab::mleval(&engine, "hold on");
		//cout << bg::wkt(polygons[i+1]) << endl;
		//cout << bg::area(polygons[i+1]) << endl;
	}
	//Polygon intersection
	/*
	Maybe there still exists bugs, but I have adapted a compromise plan
	*/
 	for (int i = 0; i < polygons.size() - 1; i++) {
		r.clear();
		if (i == 0) {
			r = polygons[0];
			t.clear();
			t = r;
		}
		else
		{
			r = t;
		}
		output.clear();
		bg::intersection(r, polygons[i], output);
		if (output.size() != 0) {
			t.clear();
			t = output[0];
		}
			
	}
	//cout << bg::wkt(t) << endl;
	//Cartesian coordinates
	Polygon_s s;
	dir.resize(t.outer().size(), 3);
	for (int i = 0; i < t.outer().size(); i++) {
		dir.row(i) << cos(t.outer()[i].y())*cos(t.outer()[i].x()), cos(t.outer()[i].y())*sin(t.outer()[i].x()), sin(t.outer()[i].y());
		s.outer().push_back(Point_s(t.outer()[i].x(), t.outer()[i].y()));
	}
	return bg::area(s);
}

double qrcode::qrArea(Engine * engine, Eigen::MatrixXd & V_qr, Eigen::MatrixXd & rot)
{
	using namespace std;
	const double pi = 3.1415926535898;
	Eigen::MatrixXd temp;
	double b;
	int index;
	// Spherical coordinate system
	namespace bg = boost::geometry;
	
	typedef	bg::model::point<double, 2, bg::cs::spherical_equatorial<bg::radian>> Point_s;
	typedef bg::model::polygon<Point_s> Polygon_s;
	Polygon_s polygon;
	double x, y, z, w;
	V_qr = (rot*V_qr.transpose()).transpose();

	/*
	For qrcode boundary
	*/
	for (int i = 0; i < V_qr.rows(); i++) {
		x = V_qr(i, 0);
		y = V_qr(i, 1);
		z = V_qr(i, 2);
		w = sqrt(pow(x, 2) + pow(y, 2));
		if (y >= 0) {
			if (x >= 0) {
				polygon.outer().push_back(Point_s(asin(y / w), asin(z)));
			}
			else {
				polygon.outer().push_back(Point_s(pi - asin(y / w), asin(z)));
			}

		}
		else {
			if (x >= 0) {
				polygon.outer().push_back(Point_s(asin(y / w), asin(z)));
			}
			else {
				polygon.outer().push_back(Point_s(-pi - asin(y / w), asin(z)));
			}

		}
	}

	x = V_qr(0, 0);
	y = V_qr(0, 1);
	z = V_qr(0, 2);
	w = sqrt(pow(x, 2) + pow(y, 2));
	if (y >= 0) {
		if (x >= 0) {
			polygon.outer().push_back(Point_s(asin(y / w), asin(z)));
		}
		else {
			polygon.outer().push_back(Point_s(pi - asin(y / w), asin(z)));
		}

	}
	else {
		if (x >= 0) {
			polygon.outer().push_back(Point_s(asin(y / w), asin(z)));
		}
		else {
			polygon.outer().push_back(Point_s(-pi - asin(y / w), asin(z)));
		}

	}
	
	return bg::area(polygon);
}

void qrcode::rotMatrix(Eigen::VectorXd & before, Eigen::VectorXd & after, Eigen::MatrixXd &rot)
{
	using namespace std;
	Eigen::Vector3d a, b,c;
	a << after(0), after(1), after(2);
	b << before(0), before(1), before(2);
	c << after(0), after(1), after(2);
	c=c.cross(b); 
	c = -c / c.norm();
	double angle = acos(a.dot(b));
	Eigen::MatrixXd I(3, 3),K(3,3);
	I.setIdentity();
	K << 0, -c(2), c(1),
		c(2), 0, -c(0),
		-c(1), c(0), 0;
	rot.resize(3, 3);
	/*rot << cos(angle) + c(0)*c(0)*(1 - cos(angle)), c(0)*c(1)*(1 - cos(angle)) - c(2)*sin(angle), c(1)*sin(angle) + c(0)*c(2)*(1 - cos(angle)),
		c(2)*sin(angle) + c(0)*c(1)*(1 - cos(angle)), cos(angle) + c(1)*c(1)*(1 - cos(angle)), -c(0)*sin(angle) + c(1)*c(2)*(1 - cos(angle)),
		-c(1)*sin(angle) + c(0)*c(2)*(1 - cos(angle)), c(0)*sin(angle) + c(1)*c(2)*(1 - cos(angle)), cos(angle) + c(2)*c(2)*(1 - cos(angle));*/
	rot = I + sin(angle)*K + (1 - cos(angle))*K*K;
	//cout << rot*b << endl;
}
