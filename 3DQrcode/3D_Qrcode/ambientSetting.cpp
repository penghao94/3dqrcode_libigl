#include "ambientSetting.h"
#include <iostream>
using namespace std;

bool qrcode::ambientSetting(igl::viewer::Viewer & viewer, Eigen::MatrixXd &V, Eigen::MatrixXi &F, float &lighting_factor)
{

	double R = 255, G = 255, B = 255;
	qrcode::ambientSetting(viewer, V, F, lighting_factor, R, G, B);
	return true;
}

bool qrcode::ambientSetting(igl::viewer::Viewer & viewer, Eigen::MatrixXd &V, Eigen::MatrixXi &F, float &lighting_factor, double &R, double &G, double &B)
{

	/*viewer.data.clear();
	Eigen::MatrixXd N;
	//igl::per_vertex_normals(V, F, N);
	//igl::per_corner_normals(V, F, 20, N);

	Eigen::MatrixXd N_faces;
	igl::per_face_normals(V, F, N_faces);

	Eigen::VectorXd AO, AO_F;
	igl::embree::ambient_occlusion(V, F, V, N, 500, AO);
	AO = 1.0 - AO.array();
	
	AO_F.resize(F.rows(), 1);

	for (int i = 0; i < AO.rows(); i++)
	{
		AO_F(i) = AO(F(i, 1));
		//AO_F(i) = (AO(F(i, 0)) + AO(F(i, 1)) + AO(F(i, 2))) / 3;
	}

	//cout << "F(1, 0):" << F(1, 0) << endl << "F:" << F << endl << "AO_F:" << AO_F << endl;

	viewer.data.set_mesh(V, F);
	viewer.data.set_normals(N_faces);

	const Eigen::RowVector3d color(R / 255, G / 255, B / 255);
	Eigen::MatrixXd C = color.replicate(F.rows(), 1);
	for (unsigned i = 0; i < C.rows(); ++i)
		C.row(i) *= AO_F(i);    //std::min<double>(AO(i)+0.2,1);
	viewer.data.set_colors(C);

	viewer.core.lighting_factor = lighting_factor;*/

	viewer.data.clear();

	Eigen::MatrixXd N_faces;
	igl::per_face_normals(V, F, N_faces);

	Eigen::MatrixXd V_B;
	V_B.resize(F.rows(), 3);
	for (int i = 0; i < V_B.rows(); i++)
	{
		V_B.row(i) = 1.0 / 3.0  * (V.row(F(i, 0)) + V.row(F(i, 1)) + V.row(F(i, 2)));
	}

	Eigen::VectorXd AO;
	igl::embree::ambient_occlusion(V, F, V_B, N_faces, 500, AO);
	AO = 1.0 - AO.array();

	viewer.data.set_mesh(V, F);
	viewer.data.set_normals(N_faces);

	const Eigen::RowVector3d color(R / 255, G / 255, B / 255);
	Eigen::MatrixXd C = color.replicate(F.rows(), 1);
	for (unsigned i = 0; i < C.rows(); ++i)
		C.row(i) *= AO(i);    //std::min<double>(AO(i)+0.2,1);
	viewer.data.set_colors(C);

	viewer.core.lighting_factor = lighting_factor;

	return true;

}
