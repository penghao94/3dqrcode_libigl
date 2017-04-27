#include "2D_visible.h"
#include <Eigen/dense>
// Define the used kernel and arrangement  
typedef CGAL::Exact_predicates_exact_constructions_kernel       Kernel;
typedef Kernel::FT												FT; 
typedef Kernel::Point_2											Point_2;
typedef Kernel::Segment_2                                       Segment_2;
typedef CGAL::Arr_segment_traits_2<Kernel>                      Traits_2;
typedef CGAL::Arrangement_2<Traits_2>                           Arrangement_2;
typedef Arrangement_2::Halfedge_const_handle                    Halfedge_const_handle;
typedef Arrangement_2::Face_handle                              Face_handle;
// Define the used visibility class 
typedef CGAL::Triangular_expansion_visibility_2<Arrangement_2,CGAL::Tag_true>  TEV;
bool qrcode::lightRegion(Eigen::RowVector2d & query_point, Eigen::MatrixXi & E, int scale, int col, std::vector<Eigen::Vector2d>& R)
{
	using namespace std;
	R.clear();
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
	Arrangement_2 output_arr;
	TEV tev(env);
	Face_handle fh = tev.compute_visibility(q, fit, output_arr);
	Arrangement_2::Ccb_halfedge_circulator curr = fh->outer_ccb();
	
	R.push_back(Eigen::Vector2d(CGAL::to_double(curr->source()->point().x()), CGAL::to_double(curr->source()->point().y())));
	while (++curr != fh->outer_ccb()) {
		R.push_back(Eigen::Vector2d(CGAL::to_double(curr->source()->point().x()), CGAL::to_double(curr->source()->point().y())));
	}
	env.clear();
	segments.clear();
	output_arr.clear();
	return true;
}

bool qrcode::lightRegion(Engine *engine, Eigen::VectorXd &src, Eigen::VectorXd &des, Eigen::Matrix4f & mode, double minZ, double t, Eigen::MatrixXd &Box, std::vector<std::vector<Eigen::MatrixXd>>& B, std::vector<std::vector<Eigen::MatrixXd>>& R)
{
	using namespace std;
	R.clear();
	Eigen::VectorXd S(4),D(4);
	vector<Eigen::Vector4d> vertex;
	vector<Eigen::MatrixXd> T;
	vector<int> flag;
	Eigen::Matrix4f model;

	model = mode.inverse().eval();
	//model=model.inverse().eval();
	//Defining query point 
	S << src(0), src(1), src(2), 1;
	S = (mode*S.cast<float>()).cast<double>();
	D << des(0), des(1), des(2), 1;
	D= (mode*D.cast<float>()).cast<double>();
//	int level = ceil((S(2) - minZ) / t);
	vector<Segment_2> segments;
	Arrangement_2 env;
	Arrangement_2 output_arr;
	for (int i = 0; i < B.size(); i++) {
		//cout << i << endl;
		double r = (minZ + i*t - S(2)) / (D(2)-S(2));
		Point_2 query(S(0) + r*(D(0) - S(0)), S(1) + r*(D(1) - S(1)));
		//Eigen::MatrixXd Q(1, 3);
		//Q << S(0) + r*(D(0) - S(0)), S(1) + r*(D(1) - S(1)), minZ + i*t;
		segments.clear();
		env.clear();
		output_arr.clear();
		//igl::matlab::mleval(&engine, "figure");
		//cout << B[i][0] << endl;
		for (int j = 0; j < B[i][0].rows(); j++) {
			//Defining the input geometry
			Point_2 s(B[i][0](j, 0), B[i][0](j, 1));
			Point_2 d(B[i][1](j, 0) , B[i][1](j, 1));
			segments.push_back(Segment_2(s, d));
			/*Eigen::MatrixXd Eg(2, 3);
			Eg.row(0) << B[i][0].row(j);
			Eg.row(1) << B[i][1].row(j);
			igl::matlab::mlsetmatrix(&engine, "E", Eg);
			igl::matlab::mleval(&engine, "plot(E(:,1),E(:,2))");
			igl::matlab::mleval(&engine, "hold on");*/
		}
		
		/*igl::matlab::mlsetmatrix(&engine, "Q", Q);
		igl::matlab::mleval(&engine, "scatter(Q(:,1),Q(:,2))");
		igl::matlab::mleval(&engine, "hold on");*/
		/*segments.push_back(Segment_2(Point_2(0, 0), Point_2(1, 0)));
		segments.push_back(Segment_2(Point_2(1, 0), Point_2(1,1)));
		segments.push_back(Segment_2(Point_2(0, 1), Point_2(1, 1)));
		segments.push_back(Segment_2(Point_2(0, 0), Point_2(0, 1)));*/
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
		vertex.push_back(Eigen::Vector4d(CGAL::to_double(curr->source()->point().x()), CGAL::to_double(curr->source()->point().y()),minZ+t*(i+1),1));
		while (++curr != fh->outer_ccb()) {
		/*	if (abs(CGAL::to_double(curr->source()->point().x()) -x)<0.000001&&abs(CGAL::to_double(curr->source()->point().y()) - y)<0.000001) {
				if (!vertex.empty()) {
					vertex.pop_back();
					flag.pop_back();
				}
			}*/
			//else
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
		//}
		Eigen::MatrixXd E(vertex.size(), 4);
		Eigen::MatrixXd G(vertex.size(), 1);
		for(int j=0;j<vertex.size();j++){
			E.row(j) << vertex[j](0), vertex[j](1), vertex[j](2), vertex[j](3);
			G.row(j) << flag[j];
		}
		/*igl::matlab::mleval(&engine, "figure");
		igl::matlab::mlsetmatrix(&engine, "temp", E);
		igl::matlab::mleval(&engine, "plot(temp(:,1),temp(:,2))");	
		igl::matlab::mleval(&engine, "hold on");*/
		//cout << E << endl;
		E = (model*(E.transpose().cast<float>())).transpose().cast<double>().block(0, 0, E.rows(), 3);
		T.push_back(E);
		T.push_back(G);
		R.push_back(T);
		vertex.clear();
		flag.clear();
		T.clear();
		E.resize(0, 0);
		G.resize(0, 0);
	}

return true;
}

