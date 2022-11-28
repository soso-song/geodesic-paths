#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/Dense>

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  R = Eigen::Matrix3d::Identity();
  t = Eigen::RowVector3d::Zero();

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(X.rows() * 3, 6);
  Eigen::VectorXd B = Eigen::VectorXd::Zero(X.rows() * 3);

  int k = X.rows();
  for (int i = 0; i < k; i++)
  {
    A(i,1) = X(i,2);
    A(i,2) = -X(i,1);
    A(i,3) = 1;
    A(i + k,0) = -X(i,2);
    A(i + k,2) = X(i,0);
    A(i + k,4) = 1;
    A(i+2*k,0) = X(i,1);
    A(i+2*k,1) = -X(i,0);
    A(i+2*k,5) = 1;
  }
  B << X.col(0) - P.col(0),
      X.col(1) - P.col(1),
      X.col(2) - P.col(2);
  // auto A1 = A.transpose() * A;
  // auto u1 = A1.inverse();
  // auto u2 = -A.transpose() * b;
  // u = u1*u2;
  Eigen::VectorXd u = (A.transpose() * A).inverse() * (-A.transpose() * B);

  Eigen::Matrix3d M;
  M << 1, -u(2), u(1),
          u(2), 1, -u(0),
          -u(1), u(0), 1;
  closest_rotation(M, R);
  t = u.tail(3);
  // std::cout << t << std::endl;
}

