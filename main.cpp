// #include "hausdorff_lower_bound.h"
#include "geodesic_remesh.h"

// #include "icp_single_iteration.h"
// #include "random_points_on_mesh.h"
// #include "point_mesh_distance.h"

#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <string>
#include <iostream>

int main(int argc, char *argv[])
{
  // predefined colors
  const Eigen::RowVector3d orange(1.0, 0.7, 0.2);
  const Eigen::RowVector3d blue(0.2, 0.3, 0.8);

  // Load input meshes
  Eigen::MatrixXd OVX,VX;
  Eigen::MatrixXi FX;
  igl::read_triangle_mesh(
      (argc > 1 ? argv[1] : "../data/pelvis-registration-complete.obj"), OVX, FX); // partial: max-registration-partial
  // igl::read_triangle_mesh(
  //   (argc>2?argv[2]:"../data/pelvis-registration-complete.obj"),VY,FY);



  // int num_samples = 100;
  // bool show_samples = true;
  // ICPMethod method = ICP_METHOD_POINT_TO_POINT;

  igl::opengl::glfw::Viewer viewer;
  const int xid = viewer.selected_data_index;
  viewer.append_mesh();
  const int yid = viewer.selected_data_index;
  // find the path
  Eigen::VectorXd path;
  find_path(OVX, FX, 1, 12, path); // from vertex 1 to vertex 12
  // viewer.data_list[yid].set_vertices(path);
  viewer.data_list[yid].set_points(path, (1. - (1. - blue.array()) * .8));

  std::cout<<R"(
    [space]  toggle animation)";

  const auto &single_iteration = [&]()
  {
    Eigen::MatrixXi F_intrinsic;
    Eigen::MatrixXd l_intrinsic;
    int i = 1;
    geodesic_remesh(VX, FX, i, F_intrinsic, l_intrinsic);

    // convert intrinsic edge length to vertices
    Eigen::MatrixXd lout2 = Eigen::MatrixXd::Zero(VX.rows(), 3);
    for (int i = 0; i < F_intrinsic.rows(); i++)
    {
      for (int j = 0; j < 3; j++)
      {
        lout2.row(F_intrinsic(i, j)) += l_intrinsic.row(i);
      }
    }

    // show intrinsic mesh in viewer
    // viewer.data_list[xid].set_mesh(VX, F_intrinsic);
    viewer.data_list[xid].set_vertices(lout2);
    viewer.data_list[xid].set_points(lout2, (1. - (1. - orange.array()) * .8));
    // viewer.data_list[xid].set_edges(VX, F_intrinsic, Eigen::RowVector3d(0.3, 0.3, 0.3));
    // viewer.data_list[xid].set_colors(l_intrinsic);
    viewer.data_list[xid].compute_normals();
    // set_points();
  };

  const auto & reset = [&]()
  {
    VX = OVX;
    viewer.data_list[xid].set_vertices(VX);
    viewer.data_list[xid].compute_normals();
    // set_points();
  };

  viewer.callback_key_pressed = 
    [&](igl::opengl::glfw::Viewer &,unsigned char key,int)->bool
  {
    switch(key)
    {
      case ' ':
        single_iteration();
        break;
      default:
        return false;
    }
    return true;
  };

  viewer.data_list[xid].set_mesh(OVX,FX);
  viewer.data_list[xid].set_colors(orange);
  // viewer.data_list[yid].set_mesh(VY,FY);
  // viewer.data_list[yid].set_colors(blue);
  reset();
  viewer.core().is_animating = false;
  viewer.data().point_size = 10;
  viewer.launch();

  return EXIT_SUCCESS;
}
