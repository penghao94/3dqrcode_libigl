#include <igl/viewer/Viewer.h>
#include <igl/triangle/triangulate.h>
#include<iostream>
// Input polygon
Eigen::MatrixXd V;
Eigen::MatrixXi E;
Eigen::MatrixXd H;

// Triangulated interior
Eigen::MatrixXd V2;
Eigen::MatrixXi F2;

int main(int argc, char *argv[])
{
  using namespace Eigen;
  using namespace std;

  // Create the boundary of a square
  V.resize(5,2);
  E.resize(5,2);
  H.resize(1,2);

  V << -1, -1, 1, -1, 1, 1,0,1, -1, 1;

  E << 0, 1, 1, 2, 2, 3, 3, 4,4,0;

  H << -1,-1;

  // Triangulate the interior
  igl::triangle::triangulate(V,E,H,"0.5",V2,F2);

  // Plot the generated mesh
  igl::viewer::Viewer viewer;
  std::cout << F2 << std::endl;
  std::cout << F2.array().nonZeros() << std::endl;
  viewer.data.set_mesh(V2,F2);
  Eigen::Vector3i a (0,0,0);
  Eigen::Vector3i b (1,0,0);
  Eigen::Vector3i c (0,1,0);
  Eigen::VectorXi d = (a-b).cross(c-a);
  delete a.data();
  cout << d << endl;
  viewer.launch();
}
