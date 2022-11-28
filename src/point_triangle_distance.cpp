#include "point_triangle_distance.h"

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  // Replace with your code
  // d = 0;
  // p = a;

  // find the normal vector n of the triangle a,b,c
  Eigen::RowVector3d ab = a-b;
  Eigen::RowVector3d ac = c-a;
  Eigen::RowVector3d n = ab.cross(ac);
  n.normalize();

  // find the projection point p of x on the plane of the triangle a,b,c
  Eigen::RowVector3d x1 = x - n.dot(x-a) * n;
  // https://math.stackexchange.com/questions/4322/check-whether-a-point-is-within-a-3d-triangle

  // if x1 is inside the triangle a,b,c, then p = x1
  
  // check if x1 is inside the triangle a,b,c
  // if((x1 - a).dot(b - a) > 0 && (x1 - b).dot(c - b) > 0 && (x1 - c).dot(a - c) > 0){
  //   d = (x - x1).norm();
  //   p = x1;
  // }
  // // check if x1 is orthogonal to ab
  // else if((x1 - a).dot(b - a) <= 0){
  //   // find the projection point p of x on the line ab
  //   Eigen::RowVector3d ab1 = b - a;
  //   ab1.normalize();
  //   Eigen::RowVector3d p1 = a + ab1.dot(x - a) * ab1;
  //   d = (x - p1).norm();
  //   p = p1;
  // }
  // // check if x1 is orthogonal to bc
  // else if((x1 - b).dot(c - b) <= 0){
  //   // find the projection point p of x on the line bc
  //   Eigen::RowVector3d bc1 = c - b;
  //   bc1.normalize();
  //   Eigen::RowVector3d p1 = b + bc1.dot(x - b) * bc1;
  //   d = (x - p1).norm();
  //   p = p1;
  // }
  // // check if x1 is orthogonal to ca
  // else if((x1 - c).dot(a - c) <= 0){
  //   // find the projection point p of x on the line ca
  //   Eigen::RowVector3d ca1 = a - c;
  //   ca1.normalize();
  //   Eigen::RowVector3d p1 = c + ca1.dot(x - c) * ca1;
  //   d = (x - p1).norm();
  //   p = p1;
  // }
  // else{
    // find the closest point on the edges
    double d1 = (x - a).norm();
    double d2 = (x - b).norm();
    double d3 = (x - c).norm();
    if(d1 < d2 && d1 < d3){
      d = d1;
      p = a;
    }else if(d2 < d1 && d2 < d3){
      d = d2;
      p = b;
    }else{
      d = d3;
      p = c;
    }
  // }

}
