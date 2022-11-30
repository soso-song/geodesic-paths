#include "geodesic_remesh.h"
#include <igl/edge_lengths.h>
#include <igl/intrinsic_delaunay_triangulation.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/exact_geodesic.h>

void geodesic_remesh(
    const Eigen::MatrixXd &V,
    const Eigen::MatrixXi &F,
    const Eigen::VectorXi &P,
    Eigen::MatrixXd &Vout,
    Eigen::MatrixXi &Fout)
{
  // convert to intrinsic mesh
  // Eigen::MatrixXd l;
  // igl::edge_lengths(V, Fin, l);
  // igl::intrinsic_delaunay_triangulation(l, Fin, lout, Fout);

  if (P.rows() < 3){
    return;
  }
  // loop over all vertices
  for (int i = 1; i < P.rows()-1; i++){
    std::cout << "next a-b-c --------" << std::endl;
    // get i th index in P
    int a = P(i-1);
    int b = P(i);
    int c = P(i+1);

    // calculate angle between a-b-c based on normal of b


    // std::cout << "angle: " << angle << std::endl;

    Eigen::MatrixXi T;
    find_inner_trangles(V, F, a, b, c, T);
    //print triangles T
    std::cout << "T: " << T << std::endl;

    Eigen::VectorXi flipP;
    flip_inner_trangles(V, T, flipP);
  }
  
}

void find_inner_trangles(
    const Eigen::MatrixXd &V,
    const Eigen::MatrixXi &F,
    const int a,
    const int b,
    const int c,
    Eigen::MatrixXi &T)
{
  // find all the triangles that contain vertex b (center)
  Eigen::MatrixXi T_AB;
  for (int j = 0; j < F.rows(); j++)
  {
    int x = F(j, 0);
    int y = F(j, 1);
    int z = F(j, 2);
    if (b == x)
    {
      T_AB.conservativeResize(T_AB.rows() + 1, 3);
      T_AB.row(T_AB.rows() - 1) << x, y, z;
    }
    else if (b == y)
    {
      T_AB.conservativeResize(T_AB.rows() + 1, 3);
      T_AB.row(T_AB.rows() - 1) << y, z, x;
    }
    else if (b == z)
    {
      T_AB.conservativeResize(T_AB.rows() + 1, 3);
      T_AB.row(T_AB.rows() - 1) << z, x, y;
    }
  }

  // break triangles in T into 2 sections
  Eigen::MatrixXi T_A;
  Eigen::MatrixXi T_B;
  int temp = a;
  bool section_A = true;
  for (int j = 0; j < T_AB.rows(); j++)
  {
    for (int i = 0; i < T_AB.rows(); i++)
    {
      if (temp == T_AB(i, 1))
      {
        if (section_A){
          T_A.conservativeResize(T_A.rows() + 1, 3);
          T_A.row(T_A.rows() - 1) << T_AB(i, 0), T_AB(i, 1), T_AB(i, 2);
          temp = T_AB(i, 2);
          if (temp == c){
            section_A = false;
          }
        } else{
          if (temp == a){
            break;
          }
          T_B.conservativeResize(T_B.rows() + 1, 3);
          T_B.row(T_B.rows() - 1) << T_AB(i, 0), T_AB(i, 1), T_AB(i, 2);
          temp = T_AB(i, 2);
        }
        continue;
      }
    }
  }

  // find the summation of the angles of T_A and T_B to check if it is inner side of a-b-c
  double angle_A = 0;
  for (int i = 0; i < T_A.rows(); i++)
  {
    int x = T_A(i, 0);
    int y = T_A(i, 1);
    int z = T_A(i, 2);
    angle_A += get_angle(V, x, y, z);
  }

  double angle_B = 0;
  for (int i = 0; i < T_B.rows(); i++)
  {
    int x = T_B(i, 0);
    int y = T_B(i, 1);
    int z = T_B(i, 2);
    angle_B += get_angle(V, x, y, z);
  }
  // print angle_A and angle_abc
  // std::cout << "angle_A: " << angle_A << std::endl;
  // std::cout << "angle_B: " << angle_B << std::endl;
  if (angle_A < angle_B)
  {
    T = T_A;
  }
  else
  {
    T = T_B;
  }
}

// "inner" means assume trangles are on the "smaller side" of a-b-c
void flip_inner_trangles( 
    const Eigen::MatrixXd &V, 
    const Eigen::MatrixXi &T,
    Eigen::VectorXi &flipP)
{
  // check if there is more than 1 triangle
  if (T.rows() < 2){
    return;
  }
  // // loop over all triangles
  // for (int i = 0; i < T.rows(); i++){
  //   // get i th triangle
  //   int a = T(i, 0);
  //   int b = T(i, 1);
  //   int c = T(i, 2);
  //   // get the other triangle
  //   int d = T(i+1, 0);
  //   int e = T(i+1, 1);
  //   int f = T(i+1, 2);
  //   // check if a-b-c and d-e-f are adjacent
  //   if (a == d && b == e){
  //     // flip a-b-c and d-e-f
  //     flipP.conservativeResize(flipP.rows()+1, 1);
  //     flipP.row(flipP.rows()-1) << a, b, c;
  //   } else if (a == e && b == f){
  //     // flip a-b-c and d-e-f
  //     flipP.conservativeResize(flipP.rows()+1, 1);
  //     flipP.row(flipP.rows()-1) << a, b, c;
  //   } else if (a == f && b == d){
  //     // flip a-b-c and d-e-f
  //     flipP.conservativeResize(flipP.rows()+1, 1);
  //     flipP.row(flipP.rows()-1) << a, b, c;
  //   }
  // }
}

double get_angle(
    const Eigen::MatrixXd &V,
    const int a,
    const int b,
    const int c)
{
  // calculate the small angle between a-b-c
  Eigen::Vector3d ab = V.row(a) - V.row(b);
  Eigen::Vector3d bc = V.row(c) - V.row(b);
  double angle = acos(ab.dot(bc) / (ab.norm() * bc.norm()));
  return angle;
}