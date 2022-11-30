#ifndef GEODESIC_REMESH_H
#define GEODESIC_REMESH_H
#include <Eigen/Core>

// Inputs:
//   V  #V by 3 list of mesh vertex positions
//   Fin  #Fin by 3 list of triangle indices into V
//   i   source vertex id
// Outputs:
//   Fout  #Fin by 3 list of output triangle indices
//   lout  #Fin by 3 list of output "half-edge lengths"
void geodesic_remesh(
    const Eigen::MatrixXd &V,
    const Eigen::MatrixXi &F,
    const Eigen::VectorXi &P,
    Eigen::MatrixXd &Vout,
    Eigen::MatrixXi &Fout);

void find_inner_trangles(
    const Eigen::MatrixXd &V,
    const Eigen::MatrixXi &F,
    const int a,
    const int b,
    const int c,
    Eigen::MatrixXi &T);

void flip_inner_trangles(
    const Eigen::MatrixXd &V,
    const Eigen::MatrixXi &T,
    Eigen::VectorXi &flipP);

double get_angle(
    const Eigen::MatrixXd &V,
    const int a,
    const int b,
    const int c);

// void find_path(
//     const Eigen::MatrixXd &V,
//     const Eigen::MatrixXi &F,
//     const int i,
//     const int j,
//     Eigen::MatrixXd &P);

// void igl_exact_geodesic_example(
//     const Eigen::MatrixXd &V,
//     const Eigen::MatrixXi &F,
//     const int i,
//     const int j,
//     Eigen::VectorXd &P);

#endif