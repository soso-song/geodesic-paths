#include "random_points_on_mesh.h"

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  X.resize(n,3);
  // for(int i = 0;i<X.rows();i++) X.row(i) = V.row(i%V.rows());

  // choose uniformly sampled random point on (V,F)
  for(int i = 0;i<X.rows();i++){
    int f = rand() % F.rows();
    double a = (double)rand() / RAND_MAX;
    double b = (double)rand() / RAND_MAX;
    if(a + b > 1){
      a = 1 - a;
      b = 1 - b;
    }
    X.row(i) = a * V.row(F(f, 0)) + b * V.row(F(f, 1)) + (1 - a - b) * V.row(F(f, 2));
  }
}

