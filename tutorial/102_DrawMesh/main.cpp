#include <igl/readOFF.h>
#include <igl/viewer/Viewer.h>
#include "tutorial_shared_path.h"
#include <iostream>
#include<Eigen/dense>
#include "histc.h"
#include <igl/cumsum.h>
#include <igl/PI.h>
Eigen::MatrixXd V(17,3);
Eigen::MatrixXi F(16,3);
int main(int argc, char *argv[])
{
	using namespace std;
	igl::viewer::Viewer viewer;
  // Load a mesh in OFF format
  //igl::readOFF(TUTORIAL_SHARED_PATH "/bunny.off", V, F);
	V << -0.196096857084488, -0.980548288503532, 0.00843069121602545,
		-0.316177562422947, -0.948602642656705, 0.0135932102536552,
		-0.706749709529654, -0.706810873551684, 0.0303848170778644,
		-0.706796847178837, 0.706763649710384, 0.0303868436352561,
		0.706840323802695, 0.706720255897348, 0.0303848078961667,
		0.948605483763931, 0.316169029936992, 0.0135934058167823,
		0.980549500121492, 0.196090836035934, 0.00842981816062213,
		0.989933007804114, 0.141406100694306, 0.00607904156353666,
		0.989930359971102, -0.141424602656382, 0.00607981668874632,
		0.919126178854959, -0.393922837309737, 0.00564496141064826,
		0.813718512135634, -0.581237651395859, 0.00499756016441025,
		0.707095500643430, -0.707104726772267, 0.00434261973177990,
		0.581228219361387, -0.813725250198736, 0.00499742003407459,
		0.393914970539362, -0.919129551651976, 0.00564475553166834,
		0.141428804990552, -0.989929761124581, 0.00607956896286035,
		-0.141411687522380, -0.989932206411479, 0.00607958398038069,
		0.00156203233676528, -0.00155338650419319, 0.999997573519730;

	F << 0, 16, 1,
		1, 16, 2,
		2, 16, 3,
		3, 16, 4,
		4, 16, 5,
		5, 16, 6,
		6, 16, 7,
		7, 16, 8,
		8, 16, 9,
		9, 16, 10,
		10, 16, 11,
		11, 16, 12,
		12, 16, 13,
		13, 16, 14,
		14, 16, 15,
		15, 16, 0;
	
	int num_samples = 3481;
  // Plot the mesh
	Eigen::MatrixXd S(num_samples, 3);
	Eigen::MatrixXd Angle(F.rows(), 10),G(F.rows(),3);
	Eigen::Vector3d A, B, C;
	for (int i = 0; i < F.rows(); i++) {
		A = V.row(F(i, 0)).transpose();
		B = V.row(F(i, 1)).transpose();
		C = V.row(F(i, 2)).transpose();
		Angle(i, 0) = acos((C.cross(A)).dot(B.cross(A)) / ((C.cross(A)).norm()*(B.cross(A)).norm()));
		Angle(i, 1) = acos((A.cross(B)).dot(C.cross(B)) / ((A.cross(B)).norm()*(C.cross(B)).norm()));
		Angle(i, 2) = acos((B.cross(C)).dot(A.cross(C)) / ((B.cross(C)).norm()*(A.cross(C)).norm()));
		Angle(i, 3) = acos(B.dot(C));
		Angle(i, 4) = acos(C.dot(A));
		Angle(i, 5) = acos(A.dot(B));
		Angle(i, 6) = Angle(i, 0) + Angle(i, 1) + Angle(i, 2) - igl::PI;
		Angle(i, 7) = (C.cross(A)).dot(B.cross(A)) / ((C.cross(A)).norm()*(B.cross(A)).norm());
		Angle(i, 8) = sqrt(1-Angle(i,7)*Angle(i,7));
		Angle(i, 9) = A.dot(B);
		G.row(i)= ((C - A.dot(C)*A)).normalized();
	}
	std::cout << Angle << std::endl;
	Eigen::VectorXd Area = Angle.col(6);
	Area /= Area.array().sum();
	Eigen::VectorXd CA, A0(Area.size() + 1);
	A0(0) = 0;
	A0.bottomRightCorner(Area.size(), 1) = Area;
	// Even faster would be to use the "Alias Table Method"
	igl::cumsum(A0, 1, CA);
	A0.resize(0);
	Area.resize(0);
	for (int i = 0; i < num_samples; i++) {
		int r = qrcode::histc(CA);
		double r1 = double(rand()) / double(RAND_MAX);
		double r2 = double(rand()) / double(RAND_MAX);
		double _A = r1*Angle(r, 6);
		
		double s = sin(_A - Angle(r, 0));
		double t = cos(_A - Angle(r, 0));
		double u = t - Angle(r, 7);
		double v = s +Angle(r, 8)*Angle(r, 9);
		double q = ((v*t - u*s)* Angle(r, 7) - v) / (v*s + u*t)* Angle(r, 8);
		Eigen::RowVectorXd a = V.row(F(r, 0));
		Eigen::RowVectorXd b = V.row(F(r, 1));
		Eigen::RowVectorXd _C = q*a + sqrt(1 - q*q)*G.row(r);
		double z = 1 - r2*(1 - _C.dot(b));
		S.row(i) = z*b + sqrt(1 - z*z)*((_C - _C.dot(b)*b)).normalized();
	}
	cout << S << endl;
  
  viewer.data.set_mesh(V, F);
  viewer.core.point_size = 5;
  viewer.data.set_points(S,Eigen::RowVector3d(1,1,1).replicate(num_samples,1));
  viewer.launch();
}
