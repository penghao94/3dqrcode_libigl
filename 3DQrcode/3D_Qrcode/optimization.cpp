#include "optimization.h"

bool qrcode::vis2gray(Engine *engine,Eigen::MatrixXd & vis, Eigen::MatrixXi & gray)
{
	Eigen::MatrixXd g(vis.rows(),vis.cols());
	igl::matlab::mlsetmatrix(&engine, "v", vis);
	igl::matlab::mleval(&engine, "g=round((log(v)+14.32)./0.05864)");
	igl::matlab::mlgetmatrix(&engine, "g", g);

	return false;
}

bool qrcode::gray2vis(Engine *engine,Eigen::MatrixXi & gray, Eigen::MatrixXd & vis)
{
	igl::matlab::mlsetmatrix(&engine, "g", gray);
	igl::matlab::mleval(&engine, "v=6.167e-11.*exp(0.1012.*g)");
	igl::matlab::mlgetmatrix(&engine, "v", vis);
	return false;
}
