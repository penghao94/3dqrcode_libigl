#ifndef SELECT_H
#define SELECT_H

#include <igl/viewer/Viewer.h>
#include <igl/unproject_onto_mesh.h>
#include <iostream>
using namespace std;

namespace qrcode {
	
	bool select(igl::viewer::Viewer& viewer, Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::MatrixXd& C);
}
#endif // !SELECT_H
