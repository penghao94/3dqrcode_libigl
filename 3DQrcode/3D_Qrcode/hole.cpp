#include "hole.h"
#include <igl/unique.h>

bool qrcode::hole(Eigen::MatrixXd & V, 
	Eigen::MatrixXi & F, 
	Eigen::MatrixXi fid, 
	Eigen::MatrixXd & _V, 
	Eigen::MatrixXi & _F)
{
	Eigen::VectorXi Fi;
	igl::unique(fid,Fi);
	_F.resize(F.rows() - Fi.size(),3);
	int j = 0;
	for (int i = 0; i < F.rows(); i++) 
	{
		if (i == Fi(j))
			j++;
		else
			_F.row(i - j) << F(i);
	}
	return false;
}
