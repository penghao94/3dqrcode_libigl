#include "cut_plane.h"
#include <Eigen/dense>
bool qrcode::cut_plane(Engine *engine, Eigen::MatrixXd & V, Eigen::MatrixXi & F, Eigen::Matrix4f & mode, int layer, std::vector<std::vector<Eigen::MatrixXd>>& B, double& minZ,double& t, Eigen::MatrixXd &Box)
{
	using namespace std;
	//typedef CGAL::Simple_cartesian<double>								Kernel; // fastest in experiments
	typedef CGAL::Exact_predicates_exact_constructions_kernel       Kernel;
	typedef Kernel::Point_3												Point;
	typedef Kernel::Plane_3												Plane;
	typedef Kernel::Segment_3											Segment;
	typedef Kernel::Triangle_3											Triangle;
	typedef CGAL::Polyhedron_3<Kernel>									Polyhedron;

	typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron>         Facet_Primitive;
	typedef CGAL::AABB_traits<Kernel, Facet_Primitive>                  Facet_Traits;
	typedef CGAL::AABB_tree<Facet_Traits>                               Facet_tree;
	typedef std::vector<Facet_tree::Object_and_primitive_id>			Intersections;
	//initiate
	Polyhedron *m_polyhedron;
	std::vector<Segment> m_cut_segments;
	Facet_tree m_facet_tree;
	Intersections intersections;
	Eigen::Matrix4f model=mode;
	//cout << mode << endl;
	model = model.inverse().eval();
	Eigen::MatrixXd S,T;
	//Translate model and getting bounding box
	Eigen::MatrixXd _V(V.rows(),4);
	Eigen::MatrixXi _F(F.rows(),3); 
	Eigen::MatrixXd tmp(2, 3);
	_V.block(0, 0, V.rows(), 3) << V;
	_F.block(0, 0, F.rows(), 3) << F;
	_V.col(3).setConstant(1);
	_V = (mode*(_V.transpose().cast<float>())).transpose().cast<double>().block(0, 0, _V.rows(), 3);
	// Find the bounding box
	Eigen::Vector3d m = _V.colwise().minCoeff();
	Eigen::Vector3d M = _V.colwise().maxCoeff();
	t = 0.999*(M(2) -m(2)) / layer;
	minZ = m(2)+0.0005*(M(2) - m(2));
	//cout << minZ << "+" << t << endl;
	Eigen::Vector3d centriod = (M + m) / 2;
	M = centriod + 2 * (M - centriod);
	m = centriod + 2 * (m - centriod);
	Box.resize(2, 3);
	Box.row(0) << M.transpose();
	Box.row(1) << m.transpose();
	// Corners of the bounding box
	Eigen::MatrixXd V_box(8, 3);
	V_box <<
		m(0), m(1), M(2),
		M(0), m(1), M(2),
		M(0), M(1), M(2),
		m(0), M(1), M(2),
		m(0), m(1), m(2),
		M(0), m(1), m(2),
		M(0), M(1), m(2),
		m(0), M(1), m(2);
	//Faces of bounding box
	Eigen::MatrixXi F_box(12, 3);
	F_box <<
		0, 1, 2,
		0, 2, 3,
		4, 6, 5,
		4, 7, 6,
		0, 5, 1,
		0, 4, 5,
		3, 2, 6,
		3, 6, 7,
		1, 5, 6,
		1, 6, 2,
		0, 7, 4,
		0, 3, 7;
	_V.conservativeResize(_V.rows() + 8, 3);
	_F.conservativeResize(_F.rows() + 12, 3);
	_V.block(V.rows(), 0, 8, 3) << V_box;
	_F.block(F.rows(), 0, 12, 3) << (F_box.array()+V.rows()).matrix();

	//Construct polyhedron
	m_polyhedron = new Polyhedron;
	for (int i = 1; i < _F.rows(); i++) {
		m_polyhedron->make_triangle(
			Point(_V(_F(i, 0), 0), _V(_F(i, 0), 1), _V(_F(i, 0), 2)),
			Point(_V(_F(i, 1), 0), _V(_F(i, 1), 1), _V(_F(i, 1), 2)),
			Point(_V(_F(i, 2), 0), _V(_F(i, 2), 1), _V(_F(i, 2), 2)));
	}
	m_facet_tree.rebuild(faces(*m_polyhedron).first, faces(*m_polyhedron).second, *m_polyhedron);
	m_facet_tree.accelerate_distance_queries();
	std::cout << "Construct AABB tree done." << std::endl;
	//Cutting model depending on layer
	for (int i = 0; i <= layer; i++) {
		intersections.clear();
		Plane plane(Point(0,0,minZ+i*t), Point(0, 1, minZ + i*t), Point(1, 1, minZ + i*t));
		m_facet_tree.all_intersections(plane, std::back_inserter(intersections));
		// Fill data structure
		m_cut_segments.clear();
		for (Intersections::iterator it = intersections.begin(),
			end = intersections.end(); it != end; ++it)
		{
			const Segment* inter_seg = CGAL::object_cast<Segment>(&(it->first));

			if (NULL != inter_seg)
			{
				if (!(inter_seg->is_degenerate())) {
					m_cut_segments.push_back(*inter_seg);
					
				}
			}
		}
		S.resize(m_cut_segments.size(), 3);
		T.resize(m_cut_segments.size(), 3);
		for (int j = 0; j < m_cut_segments.size(); j++) {
				S.row(j) << CGAL::to_double( m_cut_segments[j].source().x()),CGAL::to_double( m_cut_segments[j].source().y()), CGAL::to_double(m_cut_segments[j].source().z());
				T.row(j) << CGAL::to_double(m_cut_segments[j].target().x()), CGAL::to_double(m_cut_segments[j].target().y()), CGAL::to_double(m_cut_segments[j].target().z());
			//tmp.row(0) << S.row(j);
			//tmp.row(1)<<T.row(j);
			/*igl::matlab::mlsetmatrix(&engine, "temp", tmp);
			igl::matlab::mleval(&engine, "plot3(temp(:,1),temp(:,2),temp(:,3))");
			igl::matlab::mleval(&engine, "hold on");*/
		}
		
		
		//cout << temp << endl << endl;
		
		std::vector<Eigen::MatrixXd> temp;
		temp.push_back(S);
		temp.push_back(T);
		B.push_back(temp);
		temp.clear();
	}
	cout << B.size() << endl;
	_V.resize(0, 0);
	_F.resize(0, 0);
	m_polyhedron->clear();
	m_facet_tree.clear();
	return true;
}
