#include <igl/readOFF.h>
#include <igl/writeOBJ.h>
#include <iostream>
#include "tutorial_shared_path.h"
#include <vector>
Eigen::MatrixXd V;
Eigen::MatrixXi F;

int main(int argc, char *argv[])
{
	using namespace std;
	Eigen::Vector3d a(0, 0, 1);
	cout << a.cross(a) << endl;
}
