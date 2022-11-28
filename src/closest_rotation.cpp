#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code
  Eigen::Matrix3d U, V, S;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  
  U = svd.matrixU();
  V = svd.matrixV();
  S = svd.singularValues().asDiagonal();
  S = Eigen::Matrix3d::Identity();
  if (U.determinant() * V.determinant() < 0) {
    S(2, 2) = -1;
  }

  R = U * S * V.transpose();
}
