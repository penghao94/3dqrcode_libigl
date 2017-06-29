#include "visPolygon.h"
#include <Eigen/dense>

bool qrcode::qrPolygon(Eigen::RowVector2d & query_point, Eigen::MatrixXi & E, int scale, int col, Eigen::MatrixXd &V_pxl, Eigen::VectorXd & V_src, Eigen::MatrixXd & rot, Polygon_c & qrpoly)
{
	using namespace std;
	double x, y;
	//Defining the input geometry
	std::vector<Segment_2> segments;
	for (int i = 0; i < E.rows(); i++) {
		Point_2 s(double(int(E(i, 0) % col)*scale), double(int(E(i, 0) / col)*scale));
		Point_2 d(double(int(E(i, 1) % col)*scale), double(int(E(i, 1) / col)*scale));
		segments.push_back(Segment_2(s, d));
	}
	//Defining the query point
	Point_2 q(query_point(1), query_point(0));
	// insert geometry into the arrangement 
	Arrangement_2 env;
	CGAL::insert_non_intersecting_curves(env, segments.begin(), segments.end());

	//Find the halfedge whose target is the query point.
	//(usually you may know that already by other means)  
	Face_handle fit;
	for (fit = env.faces_begin(); fit != env.faces_end(); ++fit) {
		if (!fit->is_unbounded()) {
			break;
			std::cout << "Holy shit" << std::endl;
		}
	}
	//visibility query
	vector<Eigen::VectorXd> R;
	Eigen::VectorXd dir(3);
	Arrangement_2 output_arr;
	TEV tev(env);
	Eigen::MatrixXd F;
	Face_handle fh = tev.compute_visibility(q, fit, output_arr);
	Arrangement_2::Ccb_halfedge_circulator curr = fh->outer_ccb();
	//The first one
	x = CGAL::to_double(curr->source()->point().y());
	y= CGAL::to_double(curr->source()->point().x());
	dir << (V_pxl.row(floor(x)*col + floor(y)) + (x - floor(x))*(V_pxl.row((floor(x) + 1)*col + floor(y)) - V_pxl.row(floor(x)*col + floor(y))) + (y - floor(y))*(V_pxl.row(floor(x)*col + floor(y) + 1) - V_pxl.row(floor(x)*col + floor(y)))).transpose() - V_src;
	dir.normalize();
	R.push_back(dir);
	//next point
	while (++curr != fh->outer_ccb()) {
		x = CGAL::to_double(curr->source()->point().y());
		y = CGAL::to_double(curr->source()->point().x());
		dir << (V_pxl.row(floor(x)*col + floor(y)) + (x - floor(x))*(V_pxl.row((floor(x) + 1)*col + floor(y)) - V_pxl.row(floor(x)*col + floor(y))) + (y - floor(y))*(V_pxl.row(floor(x)*col + floor(y) + 1) - V_pxl.row(floor(x)*col + floor(y)))).transpose() - V_src;
		dir.normalize();
		R.push_back(dir);
	}
	E.resize(R.size(), 3);
	for (int i = 0; i < R.size(); i++) {
		F.row(i) << R[i].transpose();
	}
	F = (rot*F.transpose()).transpose();
	//convert to spherical coordinate
	Eigen::VectorXd a(2);
	a << 1, 1;
	for (int i = 0; i < R.size(); i++) {
		dir = F.row(i).transpose();
		spi2sphere(dir, a, qrpoly);
	}
	dir = F.row(F.rows()-1).transpose();
	spi2sphere(dir, a, qrpoly);

	dir.resize(0);
	a.resize(0);
	R.clear();
	env.clear();
	segments.clear();
	output_arr.clear();
	return true;
}

bool qrcode::slPolygon(Eigen::VectorXd & src, Eigen::VectorXd & des, Eigen::Matrix4f & mode, const double minZ, const double t, const int i, Eigen::MatrixXd & Box, std::vector<Eigen::MatrixXd>& B, Eigen::MatrixXd & rot, Polygon_c & slpoly)
{
	using namespace std;
	Eigen::VectorXd S(4), D(4);
	Eigen::Matrix4f model;
	model = mode.inverse().eval();
	//Defining query point 
	S << src(0), src(1), src(2), 1;
	S = (mode*S.cast<float>()).cast<double>();
	D << des(0), des(1), des(2), 1;
	D = (mode*D.cast<float>()).cast<double>();

	vector<Eigen::Vector4d> vertex;
	vector<Eigen::MatrixXd> T;
	vector<int> flag;

	vector<Segment_2> segments;
	Arrangement_2 env;
	Arrangement_2 output_arr;

	double r = (minZ + i*t - S(2)) / (D(2) - S(2));
	Point_2 query(S(0) + r*(D(0) - S(0)), S(1) + r*(D(1) - S(1)));
	Eigen::MatrixXd Q(1, 3);
	//Q << S(0) + r*(D(0) - S(0)), S(1) + r*(D(1) - S(1)), minZ + i*t;
	//boundary
	for (int j = 0; j < B[0].rows(); j++) {
		//Defining the input geometry
		Point_2 s(B[0](j, 0), B[0](j, 1));
		Point_2 d(B[1](j, 0), B[1](j, 1));
		segments.push_back(Segment_2(s, d));
	}

	//CGAL::insert_non_intersecting_curves(env, segments.begin(), segments.end());
	CGAL::insert_curves(env, segments.begin(), segments.end());
	//Find the halfedge whose target is the query point.
	//(usually you may know that already by other means)  
	Face_handle fit;
	for (fit = env.faces_begin(); fit != env.faces_end(); ++fit) {
		if (!fit->is_unbounded()) {
			break;
			std::cout << "Holy shit" << std::endl;
		}
	}
	TEV tev(env);
	Face_handle fh = tev.compute_visibility(query, fit, output_arr);
	double x, y;
	Arrangement_2::Ccb_halfedge_circulator curr = fh->outer_ccb();
	x = CGAL::to_double(curr->source()->point().x());
	y = CGAL::to_double(curr->source()->point().y());
	if (x == Box(0, 0) || x == Box(0, 1) || y == Box(0, 1) || y == Box(1, 1)) {
		flag.push_back(1);
	}
	else {
		flag.push_back(0);
	}
	vertex.push_back(Eigen::Vector4d(CGAL::to_double(curr->source()->point().x()), CGAL::to_double(curr->source()->point().y()), minZ + t*i, 1));
	while (++curr != fh->outer_ccb()) {
		Segment_2 check(curr->source()->point(), curr->target()->point());
		if (!check.is_degenerate()) {
			x = CGAL::to_double(curr->source()->point().x());
			y = CGAL::to_double(curr->source()->point().y());
			if (x == Box(0, 0) || x == Box(0, 1) || y == Box(0, 1) || y == Box(1, 1)) {
				flag.push_back(1);
			}
			else {
				flag.push_back(0);
			}
			vertex.push_back(Eigen::Vector4d(CGAL::to_double(curr->source()->point().x()), CGAL::to_double(curr->source()->point().y()), minZ + t*i, 1));
		}

	}
	Eigen::MatrixXd E(vertex.size(), 4);
	for (int j = 0; j<vertex.size(); j++) {
		E.row(j) << vertex[j](0), vertex[j](1), vertex[j](2), vertex[j](3);
	}
	
	E = ((model*(E.transpose().cast<float>())).transpose().cast<double>().block(0, 0, E.rows(), 3)-src.transpose().replicate(E.rows(),1)).normalized();
	E = (rot*E.transpose()).transpose();
	//convert to spherical coordinate
	Eigen::VectorXd c(2);
	Eigen::VectorXd dir(3);
	c << 1, 1;
	for (int i = 0; i < E.rows(); i++) {
		dir = E.row(E.rows() - 1 - i).transpose();
		spi2sphere(dir, c,flag[E.rows() - 1 - i], slpoly);
	}
	dir = E.row(E.rows() - 1 ).transpose();
	spi2sphere(dir, c, flag[E.rows() - 1], slpoly);
	vertex.clear();
	segments.clear();
	env.clear();
	output_arr.clear();
	flag.clear();
	E.resize(0, 0);
	return true;
}

bool qrcode::spi2sphere(Eigen::VectorXd & dir, Eigen::VectorXd & a, Polygon_c & polygon)
{
	#define PI 3.1415926535898
	
	double b;
	double x1, y1, z1, w1;
	x1 = dir(0);
	y1 = dir(1);
	z1 = dir(2);
	w1 = sqrt(pow(x1, 2) + pow(y1, 2));
	if (y1 >= 0) {
		if (x1 >= 0) {
			polygon.outer().push_back(Point_c(asin(y1 / w1), asin(z1)));
			a << asin(y1 / w1), asin(z1);
		}
		else {
			if (a(0) < 0) {
				b = (asin(z1) - a(1)) / (2 * PI - PI + asin(y1 / w1) + a(0))*(PI + a(0)) + a(1);
				polygon.outer().push_back(Point_c(-PI, b));
				polygon.outer().push_back(Point_c(-PI, 1.57079632));
				polygon.outer().push_back(Point_c(PI, 1.57079632));
				polygon.outer().push_back(Point_c(PI, b));
				polygon.outer().push_back(Point_c(PI - asin(y1 / w1), asin(z1)));
			}
			else
				polygon.outer().push_back(Point_c(PI - asin(y1 / w1), asin(z1)));
			a << PI - asin(y1 / w1), asin(z1);
		}

	}
	else {
		if (x1 >= 0) {
			polygon.outer().push_back(Point_c(asin(y1 / w1), asin(z1)));
			a << asin(y1 / w1), asin(z1);
		}
		else {
			polygon.outer().push_back(Point_c(-PI - asin(y1 / w1), asin(z1)));
			a << -PI - asin(y1 / w1), asin(z1);
		}
	}

	return true;
}

bool qrcode::spi2sphere(Eigen::VectorXd & dir, Eigen::VectorXd & c, const double flag, Polygon_c & polygon)
{
	#define PI 3.1415926535898
	double x, y, z, w;
	double f;

	x = dir(0);
	y = dir(1);

	if (flag == 0)
		z = dir(2);
	else
		z = -0.9999999;
	w = sqrt(pow(x, 2) + pow(y, 2));

	if (y >= 0) {
		if (x >= 0) {
			polygon.outer().push_back(Point_c(asin(y / w), asin(z)));
			c << asin(y / w), asin(z);
		}
		else {
			if (c(0) < 0) {
				f = (asin(z) - c(1)) / (2 * PI - PI + asin(y / w) + c(0))*(PI + c(0)) + c(1);
				polygon.outer().push_back(Point_c(-PI, f));
				polygon.outer().push_back(Point_c(-PI, 1.57079632));
				polygon.outer().push_back(Point_c(PI, 1.57079632));
				polygon.outer().push_back(Point_c(PI, f));
				polygon.outer().push_back(Point_c(PI - asin(y / w), asin(z)));
			}
			else
				polygon.outer().push_back(Point_c(PI - asin(y / w), asin(z)));
			c << PI - asin(y / w), asin(z);
		}

	}
	else {
		if (x >= 0) {
			polygon.outer().push_back(Point_c(asin(y / w), asin(z)));
			c << asin(y / w), asin(z);
		}
		else {
			polygon.outer().push_back(Point_c(-PI - asin(y / w), asin(z)));
			c << -PI - asin(y / w), asin(z);
		}

	}
	return true;
}
