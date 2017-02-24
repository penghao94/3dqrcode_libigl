#pragma once
#ifndef IMGTOMESH_H_
#define IMGTOMESH_H_
#include <iostream>
#include <igl/viewer/Viewer.h>
#include <igl/unproject_in_mesh.h>
namespace qrcode {
	
	//************************************
	// Method:    img_to_mesh
	// FullName:  qrcode::img_to_mesh
	// Access:    public 
	// Returns:   bool
	// Qualifier:
	// Parameter: igl::viewer::Viewer & viewer
	// Parameter: Eigen::MatrixXd & V
	// Parameter: Eigen::MatrixXi & F
	// Parameter: Eigen::MatrixXi & D
	// Parameter: Eigen::MatrixXi & fid
	// Parameter: Eigen::MatrixXd & _V
	// Parameter: Eigen::MatrixXi & _F
	//************************************
	bool img_to_mesh(
		igl::viewer::Viewer &viewer,
		Eigen::MatrixXd &V,
		Eigen::MatrixXi &F,
		Eigen::MatrixXi &D,
		Eigen::MatrixXi &fid,
		Eigen::MatrixXd &_V,
		Eigen::MatrixXi &_F,
		Eigen::MatrixXd &_C
	);

	//************************************
	// Method:    img_to_facet
	// FullName:  qrcode::img_to_facet
	// Access:    public 
	// Returns:   bool
	// Qualifier:
	// Parameter: Eigen::MatrixXi & D
	// Parameter: Eigen::MatrixXi & F
	// Parameter: Eigen::MatrixXd & C
	//************************************
	bool img_to_facet(Eigen::MatrixXi &D, Eigen::MatrixXi &F, Eigen::MatrixXd &C);
}
#endif // !IMGTOMESH_H_
