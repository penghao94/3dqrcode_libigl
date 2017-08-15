#include "illuminate_map.h"
#include "random_point_on_spherical_mesh.h"
#include <igl/barycenter.h>
#include <igl/Hit.h>
#include <igl/parallel_for.h>
#include <vector>

void qrcode::illumin_origin(Eigen::MatrixXd & V, Eigen::MatrixXi & F, Eigen::Matrix4f &mode,int num_samples, int distance, Eigen::RowVectorXd &centriod, Eigen::MatrixXd & P)
{
	Eigen::Matrix4f model;
	model = mode.inverse().eval();
	Eigen::MatrixXd _V;
	_V.setOnes(V.rows(), 4);
	_V.block(0, 0, V.rows(), 3) << V;
	_V = (mode*(_V.transpose().cast<float>())).transpose().cast<double>().block(0, 0, _V.rows(), 3);
	Eigen::MatrixXd BC;
	igl::barycenter(_V,F,BC);
	centriod = 0.5*(BC.colwise().maxCoeff() + BC.colwise().minCoeff()).eval();
	P.resize(num_samples, 3);
	for (int i = 0; i < num_samples; i++) {
		P.row(i) = random_dir();
	}
	P = centriod.replicate(num_samples, 1) + distance*P;
	P.conservativeResize(num_samples, 4);
	P.col(3).setConstant(1);
	P = (model*(P.transpose().cast<float>())).transpose().cast<double>().block(0, 0, P.rows(), 3);
}

void qrcode::illuminate_map(igl::embree::EmbreeIntersector& ei, Eigen::MatrixXd & V, Eigen::MatrixXi & F, Eigen::MatrixXd & org, Eigen::MatrixXd & des, Eigen::VectorXd & map)
{
	using namespace Eigen;
	int n = org.rows();
	//map has the same size of origin point
	map.resize(n);
	//initiate Embree and construct shoot ray function
	//igl::embree::EmbreeIntersector ei;
	ei.init(V.cast<float>(), F);
	
	const auto &shoot_ray = [&ei](
		const Eigen::Vector3f& s,
		const Eigen::Vector3f& dir)->bool
	{
		igl::Hit hit;
		const float tnear = 1e-4f;
		return ei.intersectRay(s, dir, hit, tnear);
	};
	// shoot rays
	const auto &inner = [&org, &des, &map, &shoot_ray](const int p)
	{
		const Vector3f d = org.row(p).cast<float>();
		int num = 0;
		for (int i = 0; i < des.rows(); i++) {
			Vector3f s = des.row(i).cast<float>();
			Vector3f dir = (d - s).normalized();
			if (!shoot_ray(s, dir))
				num++;
		}
		map(p) = double(num) / des.rows();  
	};
	igl::parallel_for(n, inner, 1000);
	ei.deinit();
}

void qrcode::is_light(igl::embree::EmbreeIntersector& ei, Eigen::MatrixXd & V, Eigen::MatrixXi & F, Eigen::VectorXd & org, Eigen::MatrixXd & des, Eigen::VectorXd & map)
{
	using namespace Eigen;
	int n = des.rows();
	//map has the same size of destination point
	map.resize(n);
	//initiate Embree and construct shoot ray function
	//igl::embree::EmbreeIntersector ei;
	ei.init(V.cast<float>(), F);
	const auto &shoot_ray = [&ei](
		const Eigen::Vector3f& s,
		const Eigen::Vector3f& dir)->bool
	{
		igl::Hit hit;
		const float tnear = 1e-4f;
		return ei.intersectRay(s, dir, hit, tnear);
	};

	// shoot rays
	const Vector3f d = org.cast<float>();
	const auto &inner = [&d, &des, &map, &shoot_ray](const int p)
	{
		Vector3f s = des.row(p).cast<float>();
		Vector3f dir = (d - s).normalized();
		if (!shoot_ray(s, dir))
			map(p) = 1;
		else
			map(p) = 0;
	};
	igl::parallel_for(n, inner, 1000);
	ei.deinit();
}
