#include "img_to_mesh.h"
#include "igl/unproject_onto_mesh.h"
#include "igl/unique.h"
#include <iostream>
#include "unproject_to_mesh.h"
bool qrcode::img_to_mesh(igl::viewer::Viewer & viewer, 
	Eigen::MatrixXd & V, 
	Eigen::MatrixXi & F, 
	Eigen::MatrixXi & D, 
	Eigen::MatrixXi & fid, 
	Eigen::MatrixXd & _V, 
	Eigen::MatrixXi & _F,
	Eigen::MatrixXd & _C,
	Eigen::MatrixXi & _E,
	Eigen::MatrixXd &_H)
{
	Eigen::Vector3f _uv;
	Eigen::Vector3d v0,v1,v2,_v;
	_V.resize(D.rows()*D.cols(),3);
	fid.resize(D.rows(),D.cols());
	_H.resize(1, 3);
	double CENT_X = viewer.core.viewport(2)/2;
	double CENT_Y = viewer.core.viewport(3)/2;
	for (int i = 0; i < D.rows(); i++) {
		for (int j = 0; j < D.cols(); j++) {
			double x = i + CENT_X - D.rows() / 2;
			double y = CENT_Y - D.cols() / 2 + j;

			if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core.view*viewer.core.model,
				viewer.core.proj, viewer.core.viewport, V, F, fid(i, j), _uv)) {
				v0 = V.row(F(fid(i, j), 0));
				v1 = V.row(F(fid(i, j), 1));
				v2 = V.row(F(fid(i, j), 2));
				_v = _uv(0)*v0 + _uv(1)*v1 + _uv(2)*v2;
				_V.row(i*D.cols() + j) << _v(0),_v(1),_v(2);
				if (i == int(D.rows() / 2) && j ==int(D.cols() / 2)) {
					_H << _v(0), _v(1), _v(2);
				}
			}
			else 
				_V.row(i*D.cols() + j) << _V.row(i*D.cols() + j-1);
		}
	}
	img_to_facet(D, _F, _C,_E);
	return true;
}

bool qrcode::img_to_mesh(igl::viewer::Viewer & viewer, Eigen::MatrixXd & V, Eigen::MatrixXi & F, Eigen::MatrixXd & D, int scale, Eigen::MatrixXi & fid, Eigen::MatrixXd & _V, Eigen::MatrixXi & _F, Eigen::MatrixXd & _C, Eigen::MatrixXi & _E, Eigen::MatrixXd & _H)
{
	Eigen::MatrixXd r, c;
	D.conservativeResize(D.rows() + 1, D.cols() + 1);
	r.setOnes(D.rows(), 1);
	c.setOnes(1, D.cols());
	D.block(0, D.cols() - 1, D.rows(), 1) << r;
	D.block(D.rows() - 1, 0, 1, D.cols()) << c;
	Eigen::Vector3f _uv;
	Eigen::Vector3d v0, v1, v2, _v;
	_V.resize(D.rows()*D.cols(), 3);
	fid.resize(D.rows(), D.cols());
	_H.resize(1, 3);
	double CENT_X = viewer.core.viewport(2) / 2;
	double CENT_Y = viewer.core.viewport(3) / 2;
	for (int i = 0; i < D.rows(); i++) {
		for (int j = 0; j < D.cols(); j++) {
			double x = i*scale + CENT_X - D.rows() / 2;
			double y = CENT_Y - D.cols() / 2 + j*scale;

			if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core.view*viewer.core.model,
				viewer.core.proj, viewer.core.viewport, V, F, fid(i, j), _uv)) {
				v0 = V.row(F(fid(i, j), 0));
				v1 = V.row(F(fid(i, j), 1));
				v2 = V.row(F(fid(i, j), 2));
				_v = _uv(0)*v0 + _uv(1)*v1 + _uv(2)*v2;
				_V.row(i*D.cols() + j) << _v(0), _v(1), _v(2);
				if (i == int(D.rows() / 2) && j == int(D.cols() / 2)) {
					_H << _v(0), _v(1), _v(2);
				}
			}
			else
				_V.row(i*D.cols() + j) << 0, 0, 0;
		}
	}
	img_to_facet(D, _F, _C, _E);
	return true;
}

int qrcode::img_to_sep_mesh(igl::viewer::Viewer & viewer, Eigen::MatrixXd & V, Eigen::MatrixXi & F, Eigen::MatrixXd & D, int scale,int acc, Eigen::MatrixXi & fid, Eigen::MatrixXd & _V, Eigen::MatrixXi & _F, Eigen::MatrixXd & _C, Eigen::MatrixXi & _E, Eigen::MatrixXd & _H, Eigen::MatrixXf &Src,Eigen::MatrixXf &Dir,Eigen::MatrixXd &L,Eigen::MatrixXi &T)
{
	using namespace std;
	Eigen::MatrixXd r, c, wht_V,blk_V,wht_C,blk_C,adj_C,V_pxl;
	Eigen::MatrixXi wht_F, blk_F,S,adj_F,a_F,IA,IC;
	Eigen::Vector3f _uv,s,dir;
	Eigen::Vector3d v0, v1, v2, _v;
	double CENT_X = viewer.core.viewport(2) / 2;
	double CENT_Y = viewer.core.viewport(3) / 2;
	double t;
	int w_blk = 0;//count of white block
	int b_blk = 0;//count of black block
	int sep_pnt = 0;//count of separate point
	int sep_idx = 0;//index of separate point
	int w_idx = 0;//index of white block
	int b_idx = 0;//index of black block
	bool isSep = true;// if is separate point or not
	int mul = scale*acc;
	int row = D.rows()*mul;
	int col = D.cols()*mul;
	V_pxl.resize(row*col, 3);
	Src.resize(row*col, 3);
	Dir.resize(row*col, 3);
	fid.resize(row+2*mul, col+2*mul);
	L.resize(row, col);
	_E.resize(4 * (D.rows() - 1), 2);
	_H.resize(1, 3);
	S.resize(D.rows()*D.cols(),1);
	T.resize(D.rows()*D.cols(), 1);
	S = S.setConstant(-1);
	T = T.setConstant(-1);
	/*
	Calculate unproject vertex
	*/
	for (int i = -mul; i < row+mul; i++) {
		for (int j = -mul; j < col+mul; j++) {

			double x = (j - col / 2+scale/2) / acc + CENT_X;
			double y = (row / 2 - i-scale/2) / acc + CENT_Y;
			bool shoot = unproject_to_mesh(Eigen::Vector2f(x,y), viewer.core.view*viewer.core.model,
				viewer.core.proj, viewer.core.viewport, V, F, s, dir, fid(i+mul, j+mul), _uv, t);
			if (i>=0&&j>=0&&i < row&&j < col) {
				v0 = V.row(F(fid(i + mul, j + mul), 0));
				v1 = V.row(F(fid(i + mul, j + mul), 1));
				v2 = V.row(F(fid(i + mul, j + mul), 2));
				_v = _uv(0)*v0 + _uv(1)*v1 + _uv(2)*v2;
				V_pxl.row(i*row + j) << _v(0), _v(1), _v(2);
				Src.row(i*row + j) << s.transpose();
				Dir.row(i*row + j) << dir.transpose();
				L(i, j) = t;
				if (i%mul == 0 && j%mul == 0) {
					int m = i / mul;
					int n = j / mul;
					if (m == int(D.rows() / 2) && n == int(D.cols() / 2)) {
						_H << _v(0), _v(1), _v(2);
					}
					if (!shoot)
						cout << "unfit coordinate " << floor(i / mul) << ":" << floor(j / mul) << endl;


					if (D(m, n) == 0 && m < D.rows() - 1 && n < D.cols() - 1) {
						w_blk++;
					}
					else if (D(m, n) == 1 && m < D.rows() - 1 && n < D.cols() - 1) {
						T.row(m*D.cols() + n) << b_blk;
						b_blk++;
						if (m != 0 && m != D.rows() - 2 && n != 0 && n != D.cols() - 2 && D(m - 1, n) == 1 && D(m, n - 1) == 1 && D(m - 1, n - 1) == 1) {
							sep_pnt++;
							isSep = false;
						}
					}

					if (!isSep)
						isSep = true;
					else {
						S(m*D.cols() + n) = sep_idx;
						sep_idx++;
					}
				}
			}
		}
	}
	/*
	Generate verticals and mesh
	*/
	wht_V.resize(D.rows()*D.cols() - sep_pnt, 3);
	wht_F.resize(2 * ((D.rows() - 1)*(D.cols() - 1) - b_blk), 3);
	wht_C.resize(2 * ((D.rows() - 1)*(D.cols() - 1) - b_blk), 3);
	blk_V.resize(4 * b_blk, 3);
	blk_F.resize(2 * b_blk, 3);
	blk_C.resize(2 * b_blk, 3);
	a_F.resize(8 * b_blk, 3);
	for (int i = 0; i < D.rows(); i++) {
		for (int j = 0; j < D.cols(); j++) {
			if (S(i*D.cols() + j) != -1)
				wht_V.row(S(i*D.cols() + j)) << V_pxl.row((i*mul)*col +j*mul);
			if (i < D.rows() - 1 && j < D.cols() - 1) {
				if (D(i, j) == 0) {
					int a = S(i*D.cols() + j,0);
					int b = S((i + 1)*D.cols() + j,0);
					int c = S(i*D.cols() + j + 1,0);
					int d = S((i + 1)*D.cols() + j + 1,0);
					wht_F.row(2 * w_idx) << a, b, c;
					wht_F.row(2 * w_idx+ 1) << b, d, c;
					wht_C.row(2 * w_idx) << 1.0, 1.0, 1.0;
					wht_C.row((2 * w_idx) + 1) << 1.0, 1.0, 1.0;
					w_idx++;

					if (i == 0)
						_E.row(j) << c, a;
					if (i == D.rows() - 2)
						_E.row(j + D.rows() - 1) << b, d;
					if (j == 0)
						_E.row(i + 2 * (D.rows() - 1)) << a, b;
					if (j == D.cols() - 2)
						_E.row(i + 3 * (D.rows() - 1)) << d, c;

				}//D(i, j) == 0
				else {
					int a = 4*b_idx;
					int b = 4*b_idx + 1;
					int c = 4*b_idx + 2;
					int d = 4*b_idx + 3;
					blk_V.row(a) << V_pxl.row(i*mul*col + j*mul);
					blk_V.row(b) << V_pxl.row((i + 1)*mul*col + j*mul);
					blk_V.row(c) << V_pxl.row(i*mul*col + (j + 1)*mul);
					blk_V.row(d) << V_pxl.row((i + 1)*mul*col + (j + 1)*mul);
					blk_F.row(2 * b_idx) << a, b, c;
					blk_F.row(2 * b_idx + 1) << b, d, c;
					blk_C.row(2 * b_idx) << 0.0, 0.0, 0.0;
					blk_C.row(2 * b_idx+1) << 0.0, 0.0, 0.0;
					if (D(i,j-1)==0)
					{
						int e = S(i*D.cols() + j,0);
						int f = S((i + 1)*D.cols() + j,0);
						int g = a + wht_V.rows();
						int h = b + wht_V.rows();
						a_F.row(8 * b_idx) << e,f,g;
						a_F.row(8 * b_idx + 1) << f,h,g;
						
					} 
					else
					{
						int e = 4 * T(i*D.cols() + j - 1) + 2 + wht_V.rows();
						int f = 4 * T(i*D.cols() + j - 1) + 3 + wht_V.rows();
						int g = a + wht_V.rows();
						int h = b + wht_V.rows();
						a_F.row(8 * b_idx) << e, f, g;
						a_F.row(8 * b_idx + 1) << f, h, g;
					}

					if (D(i, j + 1) == 0) {
						int e = c + wht_V.rows();
						int f = d + wht_V.rows();
						int g = S(i*D.cols() + j + 1);
						int h = S((i + 1)*D.cols() + j + 1);
						a_F.row(8 * b_idx+2) << e, f, g;
						a_F.row(8 * b_idx + 3) << f, h, g;
					}
					else {
						int e = c + wht_V.rows();
						int f = d + wht_V.rows();
						int g= 4 * T(i*D.cols() + j + 1) + 0 + wht_V.rows();
						int h= 4 * T(i*D.cols() + j + 1) + 1 + wht_V.rows();
						a_F.row(8 * b_idx + 2) << e, f, g;
						a_F.row(8 * b_idx + 3) << f, h, g;
					}

					if (D(i - 1, j) == 0) {
						int e = S(i*D.cols() + j);
						int f = a + wht_V.rows();
						int g= S(i*D.cols() + j+1);
						int h = c + wht_V.rows();
						a_F.row(8 * b_idx + 4) << e, f, g;
						a_F.row(8 * b_idx + 5) << f, h, g;
					}

					else {
						int e = 4 * T((i - 1)*D.cols() + j) + 1 + wht_V.rows();
						int f = a + wht_V.rows();
						int g = 4 * T((i - 1)*D.cols() + j) + 3 + wht_V.rows();
						int h = c + wht_V.rows();
						a_F.row(8 * b_idx + 4) << e, f, g;
						a_F.row(8 * b_idx + 5) << f, h, g;
					}

					if (D(i+1,j)==0)
					{
						int e = b + wht_V.rows();
						int f= S((i + 1)*D.cols() + j);
						int g = d + wht_V.rows();
						int h = S((i + 1)*D.cols() + j + 1);
						a_F.row(8 * b_idx + 6) << e, f, g;
						a_F.row(8 * b_idx + 7) << f, h, g;
					} 
					else
					{
						int e = b + wht_V.rows();
						int f = 4 * T((i + 1)*D.cols() + j) + 0 + wht_V.rows();
						int g = d + wht_V.rows();
						int h = 4 * T((i + 1)*D.cols() + j) + 2 + wht_V.rows();
						a_F.row(8 * b_idx + 6) << e, f, g;
						a_F.row(8 * b_idx + 7) << f, h, g;
					}
					b_idx++;
				}//D(i, j) == 1
			}
		}		
	}
	/*
	merge white and black block to a matrix
	*/
	igl::unique_rows(a_F, adj_F, IA, IC);
	adj_C.setOnes(adj_F.rows(), 3);
	
	_V.resize(wht_V.rows() + blk_V.rows(), 3);
	_F.resize(wht_F.rows() + blk_F.rows()+adj_F.rows(), 3);
	_C.resize(wht_C.rows() + blk_C.rows()+adj_C.rows(), 3);
	_V.block(0, 0, wht_V.rows(), 3)= wht_V;
	_V.block(wht_V.rows(), 0, blk_V.rows(), 3) = blk_V;
	_F.block(0, 0, wht_F.rows(), 3) = wht_F;
	_F.block(wht_F.rows(), 0, blk_F.rows(), 3) << (blk_F.array()+wht_V.rows()).matrix();
	_F.block(wht_F.rows() + blk_F.rows(), 0, adj_F.rows(), 3)=adj_F;
	_C.block(0, 0, wht_C.rows(), 3) = wht_C;
	_C.block(wht_C.rows(), 0, blk_C.rows(), 3) = blk_C;
	_C.block(wht_C.rows() + blk_C.rows(), 0, adj_C.rows(), 3) = adj_C;
	return wht_V.rows();
}

bool qrcode::img_to_facet(Eigen::MatrixXi & D, Eigen::MatrixXi & F, Eigen::MatrixXd & C,Eigen::MatrixXi &E)
{
	F.resize(2 * (D.rows() - 1)*(D.cols() - 1), 3);
	C.resize(2 * (D.rows() - 1)*(D.cols() - 1), 3);
	E.resize(4*(D.rows()-1),2);
	for (int i = 0; i < D.rows()-1; i++) {
		for (int j = 0; j < D.cols()-1; j++) {
			int a = i*D.cols() + j;
			int b = (i + 1)*D.cols() + j;
			int c = i*D.cols() + j + 1;
			int d = (i + 1)*D.cols() + j + 1;
			F.row(2 * (i*(D.cols() - 1) + j)) << a,b ,c;
			F.row(2 * (i*(D.cols() - 1) + j) +1) <<b,d, c;
			if (D(i,j) + D(i+1,j) +D(i,j+1) < 1)
				C.row(2*(i*(D.cols() - 1) + j)) << 0, 0, 0;
			else
				C.row(2 * (i*(D.cols() - 1) + j)) << 1.0, 1.0, 1.0;

			if (D(i+1, j+1) + D(i + 1, j) + D(i, j + 1)< 1)
				C.row(2 * (i*(D.cols() - 1) + j) +1) << 0, 0, 0;
			else
				C.row(2 * (i*(D.cols() - 1) + j) +1) << 1.0, 1.0, 1.0;

			if (i == 0) 
				E.row(j) << c, a;
			if (i == D.rows() - 2)
				E.row(j + D.rows() - 1) << b, d;
			if (j == 0)
				E.row(i + 2 * (D.rows() - 1)) << a, b;
			if (j == D.cols() - 2)
				E.row(i + 3 * (D.rows() - 1)) << d, c;
			
		}
	}
	return true;
}

bool qrcode::img_to_facet(Eigen::MatrixXd & D, Eigen::MatrixXi & F, Eigen::MatrixXd & C, Eigen::MatrixXi & E)
{
	
	
	F.resize(2 * (D.rows() - 1)*(D.cols() - 1), 3);
	C.resize(2 * (D.rows() - 1)*(D.cols() - 1), 3);
	E.resize(4 * (D.rows() - 1), 2);
	for (int i = 0; i < D.rows() - 1; i++) {
		for (int j = 0; j < D.cols() - 1; j++) {
			int a = i*D.cols() + j;
			int b = (i + 1)*D.cols() + j;
			int c = i*D.cols() + j + 1;
			int d = (i + 1)*D.cols() + j + 1;
			F.row(2 * (i*(D.cols() - 1) + j)) << a, b, c;
			F.row(2 * (i*(D.cols() - 1) + j) + 1) << b, d, c;
			if (D(i, j) == 1.0) {
				C.row(2 * (i*(D.cols() - 1) + j)) << 0, 0, 0;
				C.row(2 * (i*(D.cols() - 1) + j) + 1) << 0, 0, 0;
			}
				
			else {
				C.row(2 * (i*(D.cols() - 1) + j)) << 1.0, 1.0, 1.0;
				C.row(2 * (i*(D.cols() - 1) + j) + 1) << 1.0, 1.0, 1.0;
			}

			if (i == 0)
				E.row(j) << c, a;
			if (i == D.rows() - 2)
				E.row(j + D.rows() - 1) << b, d;
			if (j == 0)
				E.row(i + 2 * (D.rows() - 1)) << a, b;
			if (j == D.cols() - 2)
				E.row(i + 3 * (D.rows() - 1)) << d, c;

		}
	}
	return true;
}

bool qrcode::img_to_sep_facet(Eigen::MatrixXd & D, Eigen::MatrixXi & F, Eigen::MatrixXd & C, Eigen::MatrixXi & E)
{
	F.resize(2 * (D.rows() - 1)*(D.cols() - 1), 3);
	C.resize(2 * (D.rows() - 1)*(D.cols() - 1), 3);
	E.resize(4 * (D.rows() - 1), 2);
	for (int i = 0; i < D.rows() - 1; i++) {
		for (int j = 0; j < D.cols() - 1; j++) {
			int a = 4*(i*(D.cols()-1)+ j);
			int b = 4 * (i*(D.cols() - 1) + j) + 1;
			int c = 4 * (i*(D.cols() - 1) + j) + 2;
			int d = 4 * (i*(D.cols() - 1) + j) + 3;
			F.row(2 * (i*(D.cols() - 1) + j)) << a, b, c;
			F.row(2 * (i*(D.cols() - 1) + j) + 1) << b, d, c;
			if (D(i, j) == 1.0) {
				C.row(2 * (i*(D.cols() - 1) + j)) << 0, 0, 0;
				C.row(2 * (i*(D.cols() - 1) + j) + 1) << 0, 0, 0;
			}

			else {
				C.row(2 * (i*(D.cols() - 1) + j)) << 1.0, 1.0, 1.0;
				C.row(2 * (i*(D.cols() - 1) + j) + 1) << 1.0, 1.0, 1.0;
			}

			if (i == 0)
				E.row(j) << c, a;
			if (i == D.rows() - 2)
				E.row(j + D.rows() - 1) << b, d;
			if (j == 0)
				E.row(i + 2 * (D.rows() - 1)) << a, b;
			if (j == D.cols() - 2)
				E.row(i + 3 * (D.rows() - 1)) << d, c;

		}
	}
	return true;
}
