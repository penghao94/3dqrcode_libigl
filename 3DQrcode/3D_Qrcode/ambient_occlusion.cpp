/*!
 * This is a project to generate QRcode using libigl. See more at https://github.com/libigl/libigl
 *
 * Copyright (C) 2017 Swanny Peng <ph1994wh@gmail.com>
 *
 * ambient_occlusion.cpp  2017/05/11 23:17
 * TODO:
 *
*/
#include "ambient_occlusion.h"
#include<igl/embree/EmbreeIntersector.h>
#include<igl/parallel_for.h>
#include "spherical_coordinate.h"
#include "radom_dir.h"
#include <igl/random_dir.h>
#include <igl/random_points_on_mesh.h>
#include "random_point_on_spherical_mesh.h"
#include <igl/writeOBJ.h>
void qrcode::ambient_occlusion(const Eigen::MatrixXd & V, const Eigen::MatrixXi & F, const Eigen::MatrixXd & P, const Eigen::MatrixXd & N, const int num_samples, Eigen::VectorXd & S)
{
	using namespace Eigen;
	igl::embree::EmbreeIntersector ei;
	ei.init(V.template cast<float>(), F.template cast<int>());
	ambient_occlusion(ei, P, N, num_samples, S);
	ei.global_deinit();
}

void qrcode::ambient_occlusion(const igl::embree::EmbreeIntersector & ei, const Eigen::MatrixXd & P, const Eigen::MatrixXd & N, const int num_samples, Eigen::VectorXd & S)
{
	const auto & shoot_ray = [&ei](
		const Eigen::Vector3f& s,
		const Eigen::Vector3f& dir)->bool
	{
		igl::Hit hit;
		const float tnear = 1e-3f;
		return ei.intersectRay(s, dir, hit, tnear);
	};
	return qrcode::ambient_occlusion(shoot_ray, P, N, num_samples, S);
}

void qrcode::ambient_occlusion(const std::function<bool(const Eigen::Vector3f&, const Eigen::Vector3f&)>& shoot_ray, const Eigen::MatrixXd & P, const Eigen::MatrixXd & N, const int num_samples, Eigen::VectorXd & S)
{
	using namespace Eigen;
	const int n = P.rows();
	// Resize output
	S.resize(n, 1);
	VectorXi hits = VectorXi::Zero(n, 1);
	// Embree seems to be parallel when constructing but not when tracing rays
	MatrixXf D = igl::random_dir_stratified(num_samples).cast<float>();
	const auto & inner = [&P, &N, &num_samples, &D, &S, &shoot_ray](const int p)
	{
		const Vector3f origin = P.row(p).cast<float>();
		const Vector3f normal = N.row(p).cast<float>();
		int num_hits = 0;
		double a = 0;
		double b = 0;
		for (int s = 0; s < num_samples; s++)
		{
			Vector3f d = D.row(s);
			if (d.dot(normal) < 0)
			{
				// reverse ray
				d *= -1;
			}
			double c = d.dot(normal);
			if (!shoot_ray(origin, d))
			{
				b += c;
			}
			a += c;
		}
		S(p) = b / a;
	};
	igl::parallel_for(n, inner, 1000);
	D.resize(0, 0);
}

void qrcode::ambient_occlusion(const Eigen::MatrixXd & V, const Eigen::MatrixXi & F, const Eigen::MatrixXd & P, const Eigen::MatrixXd & N, const std::vector<qrcode::Mesh>& M, Eigen::VectorXd &A, Eigen::VectorXd & S)
{
	using namespace Eigen;
	igl::embree::EmbreeIntersector ei;
	ei.init(V.cast<float>(), F);
   ambient_occlusion(ei, P, N, M, A,S);
   ei.global_deinit();
}

void qrcode::ambient_occlusion(const igl::embree::EmbreeIntersector & ei, const Eigen::MatrixXd & P, const Eigen::MatrixXd & N, const std::vector<qrcode::Mesh>& M, Eigen::VectorXd &A, Eigen::VectorXd & S)
{
	const auto & shoot_ray = [&ei](
		const Eigen::Vector3f& s,
		const Eigen::Vector3f& dir)->bool
	{
		igl::Hit hit;
		const float tnear = 1e-3f;
		return ei.intersectRay(s, dir, hit, tnear);
	};
	return ambient_occlusion(shoot_ray, P, N, M,A, S);
}
#include<igl/matlab/MatlabWorkspace.h>
void qrcode::ambient_occlusion(const std::function<bool(const Eigen::Vector3f&, const Eigen::Vector3f&)>& shoot_ray, const Eigen::MatrixXd & P, const Eigen::MatrixXd & N, const std::vector<qrcode::Mesh>& M, Eigen::VectorXd &A, Eigen::VectorXd & S)
{
	using namespace Eigen;
	using namespace std;
	assert(P.rows() == N.rows() && P.rows() == M.size());
	const int n = P.rows();
	//Resize output
	S.resize(n,1);
	std::vector<Eigen::MatrixXd> B(n);
	A.resize(n);
	VectorXd Ar(n);
	VectorXd axis(3);
	axis << 0, 0, 1;
	const auto &rander = [&P,&M, &B, &N,&axis,&A,&Ar](const int p) {
		const Vector3f origin = P.row(p).cast<float>();
		const MatrixXd v = M[p].V;
		const MatrixXi f = M[p].F;
		const VectorXd normal = N.row(p).transpose();
		const MatrixXd rot = qrcode::rot(normal, axis);
		MatrixXd box(2, 2);
		double m;
		A(p) = qrcode::Box(origin,v,rot,box);
		const int sample = 10 * log10(A(p) / 2 / igl::PI / 0.000001);
		Ar(p) =  igl::PI / A(p)*sample*sample;
		qrcode::random_points_on_spherical_mesh(origin,v, f, sample*sample,p, B[p]);
	};
	igl::parallel_for(n, rander, 1000);
	const auto & inner = [&P, &N,&B, &S, &Ar,&A,&shoot_ray](const int p)
	{
			igl::matlab::MatlabWorkspace mw;
			const Vector3f origin = P.row(p).template cast<float>();
			const Vector3f normal = N.row(p).template cast<float>();
			double a = 0;
			double b = 0;
			for (int s = 0; s < B[p].rows(); s++)
			{
				Vector3f d = B[p].row(s).cast<float>();
				double c = d.dot(normal);
				if (!shoot_ray(origin, d))
				{
					b += c;
					a += 1;
				}
			}
			S(p) = b / Ar(p);
			
	};

	igl::parallel_for(n, inner, 1000);
	Ar.resize(0);
	B.clear();
	B.swap(vector<MatrixXd>());
}
