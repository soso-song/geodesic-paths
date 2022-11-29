#include "geodesic_remesh.h"
#include <igl/edge_lengths.h>
#include <igl/intrinsic_delaunay_triangulation.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/exact_geodesic.h>

void geodesic_remesh(
    const Eigen::MatrixXd &V,
    const Eigen::MatrixXi &Fin,
    const int i,
    Eigen::MatrixXi &Fout,
    Eigen::MatrixXd &lout)
{
  // convert to intrinsic mesh
  // Eigen::MatrixXi F;
  Eigen::MatrixXd l;
  igl::edge_lengths(V, Fin, l);
  
  // Eigen::MatrixXd l_intrinsic;
  // Eigen::MatrixXi F_intrinsic;
  // igl::intrinsic_delaunay_triangulation(l, Fin, l_intrinsic, F_intrinsic);
  igl::intrinsic_delaunay_triangulation(l, Fin, lout, Fout);
  
}

void find_path(
    const Eigen::MatrixXd &V,
    const Eigen::MatrixXi &F,
    const int i,
    const int j,
    Eigen::VectorXd &P)
{
  // // find shortest path
  // Eigen::MatrixXd D;
  // Eigen::MatrixXi P;
  // igl::shortest_edge_and_midpoint(V, F, D, P);
  // // std::cout << "D: " << D << std::endl;
  // // std::cout << "P: " << P << std::endl;
  // find shortest path between i and j
  Eigen::VectorXi VS, FS, VT, FT;
  VS.resize(1);
  VT.resize(1);
  VS << i;
  VT << j;

  igl::exact_geodesic(V, F, VS, FS, VT, FT, P);
}

    // Undefined symbols for architecture x86_64:
    // "geodesic_remesh(
    //   Eigen::Matrix<double, -1, -1, 0, -1, -1> const&,
    //   Eigen::Matrix<int, -1, -1, 0, -1, -1> const&,
    //   int,
    //   Eigen::Matrix<int, -1, -1, 0, -1, -1>&,
    //   Eigen::Matrix<double, -1, -1, 0, -1, -1>&)", referenced from:
    //       main::$_0::operator()(igl::opengl::glfw::Viewer&) const in main.cpp.o
    // ld: symbol(s) not found for architecture x86_64
    // clang: error: linker command failed with exit code 1 (use -v to see invocation)
    // make[2]: *** [registration] Error 1
    // make[1]: *** [CMakeFiles/registration.dir/all] Error 2
    // make: *** [all] Error 2