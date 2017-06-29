#include "random_point_on_spherical_mesh.h"
#include<igl/PI.h>
#include<igl/cumsum.h>
#include "histc.h"
#include<Eigen/dense>
#include <igl/PI.h>
void qrcode::random_points_on_spherical_mesh(const Eigen::Vector3f& origin, const Eigen::MatrixXd & V, const Eigen::MatrixXi & F, int num_samples, int p,Eigen::MatrixXd & S)
{
	using namespace std;
	Eigen::MatrixXd _V = (V - origin.cast<double>().transpose().replicate(V.rows(), 1)).rowwise().normalized();
	S.resize(num_samples, 3);
	Eigen::MatrixXd Angle(F.rows(), 10), G(F.rows(), 3);
	Eigen::Vector3d A, B, C;
	for (int i = 0; i < F.rows(); i++) {
		A = _V.row(F(i, 0)).transpose();
		B = _V.row(F(i, 1)).transpose();
		C = _V.row(F(i, 2)).transpose();
		Angle(i, 0) = acos(refine((C.cross(A)).dot(B.cross(A)) / ((C.cross(A)).norm()*(B.cross(A)).norm())));
		Angle(i, 1) = acos(refine((A.cross(B)).dot(C.cross(B)) / ((A.cross(B)).norm()*(C.cross(B)).norm())));
		Angle(i, 2) = acos(refine((B.cross(C)).dot(A.cross(C)) / ((B.cross(C)).norm()*(A.cross(C)).norm())));
		Angle(i, 3) = acos(refine(B.dot(C)));
		Angle(i, 4) = acos(refine(C.dot(A)));
		Angle(i, 5) = acos(refine(A.dot(B)));
		Angle(i, 6) = Angle(i, 0) + Angle(i, 1) + Angle(i, 2) - igl::PI;
		Angle(i, 7) = (C.cross(A)).dot(B.cross(A)) / ((C.cross(A)).norm()*(B.cross(A)).norm());
		Angle(i, 8) = sqrt(1 - Angle(i, 7)*Angle(i, 7));
		Angle(i, 9) = A.dot(B);
		G.row(i) = ((C - A.dot(C)*A)).normalized();
		
	}
	Eigen::VectorXd Area = Angle.col(6);
	
	Area /= Area.sum();
	Eigen::VectorXd CA, A0(Area.size() + 1);
	A0(0) = 0;
	A0.bottomRightCorner(Area.size(), 1) = Area;
	// Even faster would be to use the "Alias Table Method"
	igl::cumsum(A0, 1, CA);
	
		
	A0.resize(0);
	Area.resize(0);
	for (int i = 0; i < num_samples; i++) {
		int r = qrcode::histc(CA);
		/*if(p==6449)
			cout << r << endl;*/
		double r1 = double(rand()) / double(RAND_MAX);
		double r2 = double(rand()) / double(RAND_MAX);
		double _A = r1*Angle(r, 6);
		
		double s = sin(_A - Angle(r, 0));
		double t = cos(_A - Angle(r, 0));
		double u = t - Angle(r, 7);
		double v = s + Angle(r, 8)*Angle(r, 9);
		double q = ((v*t - u*s)* Angle(r, 7) - v) / (v*s + u*t)* Angle(r, 8);
		Eigen::RowVectorXd a = _V.row(F(r, 0));
		Eigen::RowVectorXd b = _V.row(F(r, 1));
		Eigen::RowVectorXd _C = q*a + sqrt(1 - q*q)*G.row(r);
		double z = 1 - r2*(1 - _C.dot(b));
		S.row(i) = z*b + sqrt(1 - z*z)*((_C - _C.dot(b)*b)).normalized();
	}
	CA.resize(0);
	Angle.resize(0, 0);
	G.resize(0, 0);
}

Eigen::Vector3d qrcode::random_dir()
{
	using namespace Eigen;
	double z = abs((double)rand() / (double)RAND_MAX*2.0 - 1.0);
	double t = (double)rand() / (double)RAND_MAX*2.0*igl::PI;
	// http://www.altdevblogaday.com/2012/05/03/generating-uniformly-distributed-points-on-sphere/
	double r = sqrt(1.0 - z*z);
	double x = r * cos(t);
	double y = r * sin(t);
	return Vector3d(x, y, z);
}
//to solve the accuracy of acos
double qrcode::refine(double r)
{
#define eps 1e-8;
	if (r - 1.0 > 0)
		return 1.0 ;
	if (r < 0.0)
		return 0.0;
}
