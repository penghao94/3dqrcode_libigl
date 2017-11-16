/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * unproject_to_mesh.cpp  2017/03/12 11:00
 * TODO:
 *
*/
#include "unproject_to_mesh.h"

bool qrcode::unproject_to_mesh(const Eigen::Vector2f & pos, const Eigen::Matrix4f & model, const Eigen::Matrix4f & proj, const Eigen::Vector4f & viewport,
	const Eigen::MatrixXd & V, const Eigen::MatrixXi & F, Eigen::Vector3f & s, Eigen::Vector3f & dir, int & fid, Eigen::Vector3f & bc, Eigen::Vector2d & t)
{
	using namespace std;
	using namespace Eigen;
	const auto & shoot_ray = [&V, &F](
	const Eigen::Vector3f& s,
	const Eigen::Vector3f& dir,
	std::vector<igl::Hit>& hits)->bool
	{
		if (!ray_mesh_intersect(s, dir, V, F, hits))
		{
		return false;
		}
		return true;
	};
return unproject_onto_mesh(pos, model, proj, viewport, shoot_ray, s, dir, fid, bc, t);
}

bool qrcode::unproject_onto_mesh(const Eigen::Vector2f & pos, const Eigen::Matrix4f & model, const Eigen::Matrix4f & proj, const Eigen::Vector4f & viewport,
 const std::function<bool(const Eigen::Vector3f&, const Eigen::Vector3f&, std::vector<igl::Hit>&)>& shoot_ray, Eigen::Vector3f & s, Eigen::Vector3f & dir, int & fid, Eigen::Vector3f & bc, Eigen::Vector2d & t)
{
	using namespace std;
	using namespace Eigen;
	igl::unproject_ray(pos, model, proj, viewport, s, dir);
	std::vector<igl::Hit> hits;
	if (!shoot_ray(s, dir, hits))
	{
		return false;
	}
	bc.resize(3);
	bc << 1.0 - hits[0].u - hits[0].v, hits[0].u, hits[0].v;
	fid = hits[0].id;
	if(hits.size()>1)
		t << hits[0].t,hits[1].t;
	else
		t << hits[0].t, hits[0].t;
}
