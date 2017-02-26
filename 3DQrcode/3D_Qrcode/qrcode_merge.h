#ifndef QRCODEMERGE_H
#define QRCODEMERGE_H
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <Eigen/Core>
namespace qrcode {
	bool merge(Eigen::MatrixXd& VA, Eigen::MatrixXi& FA, 
		Eigen::MatrixXd& VB, Eigen::MatrixXi& FB, 
		Eigen::MatrixXd& VC, Eigen::MatrixXi& FC);
}
#endif
