#include "geodesic_remesh.h"
#include <igl/edge_lengths.h>
#include <igl/intrinsic_delaunay_triangulation.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/exact_geodesic.h>

void geodesic_remesh(
    const Eigen::MatrixXd &V,
    Eigen::MatrixXi &F,
    const Eigen::VectorXi &P,
    Eigen::MatrixXd &Vout,
    Eigen::MatrixXi &Fout,
    Eigen::VectorXi &flipPathIndex,
    Eigen::MatrixXi &process_path)
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
    bool is_tb = find_inner_triangles(V, F, a, b, c, T);
    //print triangles T
    // std::cout << "T: " << T << std::endl;

    flip_inner_triangles(V, F, a, b, c, is_tb, T, flipPathIndex, process_path);
  }
}

// "inner" means assume triangles are on the "smaller side" of a-b-c
bool flip_inner_triangles(
    const Eigen::MatrixXd &V,
    Eigen::MatrixXi &F,
    const int s,
    const int m,
    const int t,
    const bool is_tb,
    Eigen::MatrixXi &T,
    Eigen::VectorXi &flipP,
    Eigen::MatrixXi &process_path)
{
  // check if there is more than 1 triangle
  if (T.rows() < 2){
    return false;
  }
  // add s as first element in flipP
  flipP.conservativeResize(flipP.rows() + 1);
  flipP(flipP.rows() - 1) = s;

  int index = 0;
  const int compare_amount = T.rows() - 1;
  for (int i = 0; i < compare_amount; i++){ // compare each pair of triangles (|T|-1 times)
    // check first opposite_angle
    int a = T(index, 0);
    int b = T(index, 1);
    int c = T(index, 2);
    int x = T(index + 1, 0);
    int y = T(index + 1, 1);
    int z = T(index + 1, 2);

    // print i
    std::cout << "i: " << i << std::endl;
    // print abcxyz
    std::cout << "abcxyz: " << a << " " << b << " " << c << " " << x << " " << y << " " << z << std::endl;
    // // print T
    // std::cout << "T: " << T << std::endl;
    // // print flipP
    // std::cout << "flipP: " << flipP << std::endl;

    if(is_tb){
      double opposite_angle = get_angle(V, a, b, c) + get_angle(V, x, z, y);
      // print opposite_angle
      std::cout << "opposite_angle b: " << opposite_angle << std::endl;
      if (opposite_angle > M_PI){
        index += 1;
        continue;
      }

      // flipP.conservativeResize(flipP.rows() + 1);
      // flipP(flipP.rows() - 1) = z;

      T(index, 1) = y;
      removeRow(T, index + 1);

      process_path.conservativeResize(process_path.rows() + 1, 2);
      process_path(process_path.rows() - 1, 0) = c;
      process_path(process_path.rows() - 1, 1) = y;

      flip_edge(a, y, b, c, F);
    }else{
      double opposite_angle = get_angle(V, a, c, b) + get_angle(V, x, y, z);
      std::cout << "opposite_angle a: " << opposite_angle << std::endl;
      if (opposite_angle > M_PI){
        index += 1;
        continue;
      }

      // flipP.conservativeResize(flipP.rows() + 1);
      // flipP(flipP.rows() - 1) = z;

      T(index, 2) = z;
      removeRow(T, index + 1);

      process_path.conservativeResize(process_path.rows() + 1, 2);
      process_path(process_path.rows() - 1, 0) = b;
      process_path(process_path.rows() - 1, 1) = z;

      flip_edge(a,b,c,z,F);
    }
    // break; // only flip once
  }
  return true;
}

bool flip_edge(
    const int a,
    const int b,
    const int c,
    const int d,
    Eigen::MatrixXi &F)
{
  // remove edge ac, add edge db

  // add trangle adc and bcd to F
  Eigen::MatrixXi newF(2, 3);
  newF << a, b, d,
          b, c, d;
  F.conservativeResize(F.rows() + 2, 3);
  F.block(F.rows() - 2, 0, 2, 3) = newF;

  // remove abc and bad
  removeRow(F, get_triangle_index(F, a, c, d));
  removeRow(F, get_triangle_index(F, a, b, c));
}

bool find_inner_triangles(
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
        if (section_A)
        {
          T_A.conservativeResize(T_A.rows() + 1, 3);
          T_A.row(T_A.rows() - 1) << T_AB(i, 0), T_AB(i, 1), T_AB(i, 2);
          temp = T_AB(i, 2);
          if (temp == c)
          {
            section_A = false;
          }
        }
        else
        {
          if (temp == a)
          {
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
    angle_A += get_angle(V, z, x, y);
    // print angle of each triangle
  }

  double angle_B = 0;
  for (int i = 0; i < T_B.rows(); i++)
  {
    int x = T_B(i, 0);
    int y = T_B(i, 1);
    int z = T_B(i, 2);
    angle_B += get_angle(V, z, x, y);
  }
  // print angle_A and angle_abc
  // std::cout << "angle_A: " << angle_A << std::endl;
  // std::cout << "angle_B: " << angle_B << std::endl;
  if (angle_A < angle_B)
  {
    T = T_A;
    return false;
  }
  else
  {
    T = T_B;
    // reverse the row order of T_B in T
    for (int i = 0; i < T.rows() / 2; i++)
    {
      T.row(i).swap(T.row(T.rows() - 1 - i));
    }
    return true;
  }
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

// code from: https://stackoverflow.com/questions/13290395/how-to-remove-a-certain-row-or-column-while-using-eigen-library-c
void removeRow(
  Eigen::MatrixXi &matrix, 
  unsigned int rowToRemove)
{
  unsigned int numRows = matrix.rows() - 1;
  unsigned int numCols = matrix.cols();

  if (rowToRemove < numRows)
    matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) = matrix.bottomRows(numRows - rowToRemove);

  matrix.conservativeResize(numRows, numCols);
}

int get_triangle_index(
    const Eigen::MatrixXi &F,
    const int a,
    const int b,
    const int c)
{
  int index = -1;
  // remove face a-b-c from F
  for (int i = 0; i < F.rows(); i++)
  {
    int x = F(i, 0);
    int y = F(i, 1);
    int z = F(i, 2);
    if (a == x && b == y && c == z)
    {
      index = i;
      break;
    }
    else if (a == y && b == z && c == x)
    {
      index = i;
      break;
    }
    else if (a == z && b == x && c == y)
    {
      index = i;
      break;
    }
  }
  return index;
}

// void remove_face(
//     const int a,
//     const int b,
//     const int c,
//     Eigen::MatrixXi &F)
// {
//   // remove face a-b-c from F
//   for (int i = 0; i < F.rows(); i++)
//   {
//     int x = F(i, 0);
//     int y = F(i, 1);
//     int z = F(i, 2);
//     if (a == x && b == y && c == z)
//     {
//       F.row(i) << -1, -1, -1;
//     }
//     else if (a == y && b == z && c == x)
//     {
//       F.row(i) << -1, -1, -1;
//     }
//     else if (a == z && b == x && c == y)
//     {
//       F.row(i) << -1, -1, -1;
//     }
//   }
// }