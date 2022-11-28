#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <igl/per_face_normals.h>
#include <iostream>

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
  // Replace with your code
  // P.resizeLike(X);
  // N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
  // for(int i = 0;i<X.rows();i++) P.row(i) = VY.row(i%VY.rows());
  // D = (X-P).rowwise().norm();

  // use point_triangle_distance to find the closest point between X and (VY,FY)
  double d, d1;
  P.resizeLike(X);
  N.resizeLike(X);
  D.resize(X.rows());
  for(int i = 0;i<X.rows();i++){
    d = std::numeric_limits<double>::infinity();
    Eigen::RowVector3d p;
    Eigen::RowVector3d n;
    for(int j = 0;j<FY.rows();j++){
      Eigen::RowVector3d p1;
      Eigen::RowVector3d n1;
      point_triangle_distance(X.row(i), VY.row(FY(j, 0)), VY.row(FY(j, 1)), VY.row(FY(j, 2)), d1, p1);
      if(d1 < d){
        d = d1;
        p = p1;
        n = n1;
      }
    }
    D(i) = d;
    P.row(i) = p;
    N.row(i) = n;
  }
}
