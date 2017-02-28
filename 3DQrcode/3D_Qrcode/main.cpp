#include "loadMesh.h"
#include "SaveMesh.h"
#include "readData.h"
#include "img_to_mesh.h"
#include "cutMesh.h"
#include <Eigen/Dense>
#include <igl/viewer/ViewerCore.h>
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <nanogui/formhelper.h>
#include <nanogui/screen.h>
#include <igl/unique.h>
#include <igl/Timer.h>
//#include <igl/jet.h>

using namespace igl;

int main(int argc, char *argv[])
{
  // Initiate viewer
  igl::viewer::Viewer viewer;
  igl::Timer timer;
  viewer.core.show_lines = false;
  Eigen::MatrixXd V,_V;
  Eigen::MatrixXi F,_F;
  Eigen::MatrixXd C,_C;
  Eigen::MatrixXi D;
  // UI Design
  viewer.callback_init = [&](igl::viewer::Viewer& viewer)
  {

	  // Add an additional menu window
	  viewer.ngui->addWindow(Eigen::Vector2i(220, 15), "I/O Operator");

	  // Add new group
	  viewer.ngui->addGroup("Load & Save");

	  // Add a button
	  viewer.ngui->addButton("Load Mesh", [&]() { 
		  qrcode::loadMesh(viewer, V, F);
	  });
	  // Add a button
	  viewer.ngui->addButton("Load	QRCode", [&]() {
		  qrcode::readData(D);
	  });
	  // Add a button
	  viewer.ngui->addButton("Save Mesh", [&]() {
		  qrcode::saveMesh(viewer, viewer.data);
		  
	  });

	 
	  viewer.ngui->addGroup("Qrcode Operator");
	  viewer.ngui->addButton("QR unproject", [&]() {
		  viewer.data.clear();
		  Eigen::MatrixXi t_F, fid;;
		  timer.start();
		  qrcode::img_to_mesh(viewer, V, F, D, fid, _V, _F, _C);
		  cout << "time = " << timer.getElapsedTime() << endl;
		  /*Eigen::VectorXi Fi;
		  igl::unique(fid, Fi);
		  t_F.resize(F.rows() - Fi.size(), 3);
		  int j = 0;
		  //cout << Fi << endl;
		  for (int i = 0; i < F.rows(); i++)
		  {
			  if (i == Fi(j))
				  j++;
			  else
				  t_F.row(i - j) << F(i);
		  }*/
		  //cout << "V" << _V << endl << "F" << _F <<endl;
		  //C= Eigen::MatrixXd::Constant(_V.rows(), 3, 0);
		  //viewer.data.set_points(_V,C);

		  viewer.data.set_mesh(_V, _F);
		  viewer.data.set_colors(_C);

		  //cout << viewer.core.model << endl;
		  //cout << fid << endl;
		  //cout << "_V" << endl << _V << endl;
		  //cout <<"D"<<endl<< _D << endl;
		 // cout << "F" << endl << _F << endl;
		 // cout << "C" << endl << _C << endl;
	  });

	  viewer.ngui->addButton("Merge	QRCode", [&]() {

		  viewer.data.clear();
		  Eigen::MatrixXd V_temp, V_union;
		  Eigen::MatrixXi F_temp, F_union;
		  Eigen::MatrixXd C_union(F_union.rows(), 3);
		  Eigen::VectorXi J;
		  timer.start();
		  igl::MeshBooleanType boolean_type(igl::MESH_BOOLEAN_TYPE_UNION);
		  //igl::copyleft::cgal::mesh_boolean(V, F, _V, _F, 2, V_temp, F_temp, J);
		  //igl::copyleft::cgal::mesh_boolean(V_temp, F_temp, _V, _F, igl::MESH_BOOLEAN_TYPE_UNION, V_union, F_union, J);
		  igl::copyleft::cgal::mesh_boolean( _V, _F, V, F, igl::MESH_BOOLEAN_TYPE_UNION, V_union, F_union, J);
		  cout << "time = " << timer.getElapsedTime() << endl;
		  viewer.data.set_mesh(V_union, F_union);
		  /*for (size_t f = 0; f < C.rows(); f++)
		  {
			  if (J(f) < F.rows())
			  {
				  C_union.row(f) = _C.row(f);
			  }
			  else
			  {
				  C_union.row(f) = Eigen::RowVector3d(0, 1, 0);
			  }
		  }*/

	  });
	  viewer.ngui->addButton("Test", [&]() {
		  viewer.data.clear();
		  igl::readOFF("F:/Graphics/git/3dqrcd_libigl/3DQrcode/3D_Qrcode/models/planexy.off", V, F);
		  viewer.data.set_mesh(V, F);
		  Eigen::MatrixXd _V;
		  Eigen::MatrixXi _F, fid;
		  fid.resize(4,2);
		  fid << 29, 18,
			  13, 2,
			  21, 5,
			  25, 9;
		  qrcode::cutMesh(V, F, fid, _V, _F);
	  });
	  // Generate menu
	  viewer.screen->performLayout();

	  return false;
  };
  // Launch the viewer
  viewer.launch();

}