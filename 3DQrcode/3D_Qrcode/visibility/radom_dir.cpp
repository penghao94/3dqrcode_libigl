#include "radom_dir.h"
#include "igl/ray_mesh_intersect.h"
#include "igl/Hit.h"
#include <igl\random_points_on_mesh.h>
#include <igl\matlab\MatlabWorkspace.h>
igl::matlab::MatlabWorkspace mw;
std::vector<Eigen::Vector3f> qrcode::random_dir(const Eigen::Vector3f &origin,const Mesh & mesh, const Eigen::MatrixXd & box, const int sample)
{
	using namespace Eigen;
	using namespace std;
	vector<Eigen::Vector3f> D;
	Eigen::MatrixXd BC;
	Eigen::VectorXi FI;
	D.resize(sample*sample);
	const double zb = sin(box(0, 1));
	const double ze = sin(box(1, 1));
	const int num = sample*sample;
	int index = 0;
	igl::Hit hit;
	Eigen::MatrixXd _V= (mesh.V - origin.cast<double>().transpose().replicate(mesh.V.rows(), 1)).rowwise().normalized();
	cout << _V << endl;
	/*while (index<num)	{
		double z = (double)rand() / (double)RAND_MAX*(zb - ze) + ze;
		double t = (double)rand() / (double)RAND_MAX*(box(0, 0) - box(1, 0)) + box(1, 0);
		double r = sqrt(1.0 - z*z);
		double x = r * cos(t);
		double y = r * sin(t);
		if (igl::ray_mesh_intersect(origin, Vector3f(float(x), float(y), float(z)), mesh.V, mesh.F, hit)) {
			D[index] << float(x), float(y), float(z);
			index++;
		}
		//cout << index << endl;
	}*/
	igl::random_points_on_mesh(num, _V, mesh.F, BC, FI);
	while (FI.minCoeff()<0|| FI.maxCoeff()> mesh.F.rows() - 1)
	{
		BC.resize(0, 0); FI.resize(0, 0);
		igl::random_points_on_mesh(num, _V, mesh.F, BC, FI);
		//cout << FI.minCoeff() << " " << FI.maxCoeff() << "  " << mesh.F.rows() - 1 << endl;
	 }
	Eigen::MatrixXd HH(num, 3);
	for (int i = 0; i < num; i++) {
		Vector3d V0 = BC(i, 0)* _V.row(mesh.F(FI(i), 0)).transpose();
		Vector3d V1 = BC(i, 1)* _V.row(mesh.F(FI(i), 1)).transpose();
		Vector3d V2 = BC(i, 2)* _V.row(mesh.F(FI(i), 2)).transpose();
		//D[i] = ((1-BC(i,0)-BC(i,1))*_V.row(mesh.F(FI(i),0))+BC(i,0)*_V.row(mesh.F(FI(i),1))+BC(i,1)*_V.row(mesh.F(FI(i),2))).transpose().cast<float>();
		D[i] = (V0 + V1 + V2).normalized().cast<float>();
		HH.row(i) = (V0 + V1 + V2).normalized().transpose();
	}
	mw.save(HH, "HH");
	mw.write("sphere.mat");
	return D;
}

Eigen::MatrixXd qrcode::random_dir(const int num_samples)
{
	Eigen::MatrixXd N(num_samples, 3);
	for (int i = 0; i < num_samples; i++) {
		double x, y, z, s = 0;
		while (s>1)
		{
			x = 2 * (double)rand() / (double)RAND_MAX - 1;
			y = 2 * (double)rand() / (double)RAND_MAX - 1;;
			z = 2 * (double)rand() / (double)RAND_MAX - 1;;
			s = x*x + y*y + z*z;
		}
		double n = sqrt(s);
		N.row(i) << x / n, y / n, z / n;
	}
	return N;
}
