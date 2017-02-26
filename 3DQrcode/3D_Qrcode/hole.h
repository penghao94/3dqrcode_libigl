#pragma once
#ifndef HOLE_H
#define HOLE_H
#include <Eigen/Core>
namespace qrcode
{
	//************************************
	// Method:    hole
	// FullName:  qrcode::hole
	// Access:    public 
	// Returns:   bool
	// Qualifier:
	// Parameter: Eigen::MatrixXd & V
	// Parameter: Eigen::MatrixXi & F
	// Parameter: Eigen::MatrixXi fid
	// Parameter: Eigen::MatrixXd & _V
	// Parameter: Eigen::MatrixXi & _F
	//************************************
	bool hole(Eigen::MatrixXd &V, 
		Eigen::MatrixXi &F, 
		Eigen::MatrixXi fid, 
		Eigen::MatrixXd &_V, 
		Eigen::MatrixXi &_F );
}
#endif // !HOLE_H

