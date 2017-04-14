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
typedef CGAL::Triangular_expansion_visibility_2<Arrangement_2>  TEV;
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

bool qrcode::lightRegion(Eigen::Vector3d & query_point, Eigen::Matrix4f & mode, double minZ, double t, std::vector<std::vector<Eigen::MatrixXd>>& B, std::vector<Eigen::MatrixXd>& R)
{
	using namespace std;
	R.clear();
	Eigen::VectorXd q(4);
	vector<Eigen::Vector4d> temp;
	Eigen::Matrix4f model;
	model = mode;
	model=model.inverse();
	//Defining query point 
	q << query_point(0), query_point(1), query_point(2), 1;
	q = (mode*q.cast<float>()).cast<double>();
	int level = ceil((q(2) - minZ) / t);
	Point_2 query(q(0), q(1));

	vector<Segment_2> segments;
	Arrangement_2 env;
	Arrangement_2 output_arr;


	for (int i = level-1; i < B.size(); i++) {
		segments.clear();
		env.clear();
		output_arr.clear();
		for (int j = 0; j < B[i][0].rows(); j++) {
			//Defining the input geometry
			Point_2 s(B[i][0](j, 0), B[i][0](j, 1));
			Point_2 d(B[i][1](j, 0), B[i][1](j, 1));
			segments.push_back(Segment_2(s, d));
		}
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

		TEV tev(env);
		Face_handle fh = tev.compute_visibility(query, fit, output_arr);

		Arrangement_2::Ccb_halfedge_circulator curr = fh->outer_ccb();
		temp.push_back(Eigen::Vector4d(CGAL::to_double(curr->source()->point().x()), CGAL::to_double(curr->source()->point().y()),minZ+t*level,1));
		while (++curr != fh->outer_ccb()) {
			temp.push_back(Eigen::Vector4d(CGAL::to_double(curr->source()->point().x()), CGAL::to_double(curr->source()->point().y()), minZ + t*level, 1));
		}
		Eigen::MatrixXd E(temp.size(), 4);
		for(int j=0;j<temp.size();j++){
			E.row(j) << temp[j];
		}
		E = (model*(E.transpose().cast<float>())).transpose().cast<double>().block(0, 0, E.rows(), 3);
		R.push_back(E);
		temp.clear();
		E.resize(0, 0);
	}

return true;
}

