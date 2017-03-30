/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * ray_intersect.cpp  2017/03/28 15:53
 * TODO:
 *
*/
#include "ray_intersect.h"
extern "C"
{
#include "igl/raytri.c"
}
bool qrcode::ray_mesh_intersect(const Eigen::VectorXd & source, const Eigen::VectorXd & dir, const Eigen::MatrixXd & V, const Eigen::MatrixXi & F, std::vector<igl::Hit>& hits)
{
	using namespace Eigen;
	using namespace std;
	// Should be but can't be const 
	Vector3d s_d = source;
	Vector3d dir_d = dir;
	hits.clear();
	// loop over all triangles
	for (int f = 0; f < F.rows(); f++)
	{
		// Should be but can't be const 
		RowVector3d v0 = V.row(F(f, 0)).template cast<double>();
		RowVector3d v1 = V.row(F(f, 1)).template cast<double>();
		RowVector3d v2 = V.row(F(f, 2)).template cast<double>();
		// shoot ray, record hit
		double t, u, v;
		if (intersect_triangle1(
			s_d.data(), dir_d.data(), v0.data(), v1.data(), v2.data(), &t, &u, &v) &&
			t > 0)
		{
			hits.push_back({ (int)f,(int)-1,(float)u,(float)v,(float)t });
		}
	}
	// Sort hits based on distance
	std::sort(
		hits.begin(),
		hits.end(),
		[](const igl::Hit & a, const igl::Hit & b)->bool { return a.t < b.t; });
	return hits.size() > 0;
}

bool qrcode::ray_mesh_intersect(const Eigen::VectorXd & source, const Eigen::VectorXd & dir, const Eigen::MatrixXd & V, const Eigen::MatrixXi & F, igl::Hit & hit)
{
	std::vector<igl::Hit> hits;
	ray_mesh_intersect(source, dir, V, F, hits);
	if (hits.size() > 0)
	{
		hit = hits.front();
		return true;
	}
	else
	{
		return false;
	}
}
