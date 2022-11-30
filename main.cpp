// #include "hausdorff_lower_bound.h"
#include "geodesic_remesh.h"

// #include "icp_single_iteration.h"
// #include "random_points_on_mesh.h"
// #include "point_mesh_distance.h"

#include <igl/unproject_onto_mesh.h>

#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <string>
#include <iostream>

struct State
{
  // Rest and transformed control points
  Eigen::MatrixXd CV;
  bool placing_handles = true;
} s;

int main(int argc, char *argv[])
{
  // predefined colors
  const Eigen::RowVector3d orange(1.0, 0.7, 0.2);
  const Eigen::RowVector3d yellow(1.0, 0.9, 0.2);
  const Eigen::RowVector3d blue(0.2, 0.3, 0.8);
  const Eigen::RowVector3d green(0.2, 0.6, 0.3);

  Eigen::MatrixXd VX, VY;
  Eigen::MatrixXi FX, FY;
  Eigen::VectorXi PathIndex;
  igl::read_triangle_mesh(
      (argc > 1 ? argv[1] : "../data/pelvis-registration-complete.obj"), VX, FX); // partial: max-registration-partial
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
  // Eigen::MatrixXd path;
  // find_path(VX, FX, 1, 12, path);
  // igl_exact_geodesic_example(VX, FX, 1, 95, path);
  // std::cout << "path: " << path << std::endl;
  // viewer.data_list[yid].set_vertices(path);
  // viewer.data_list[yid].set_points(path, (1. - (1. - blue.array()) * .8));

  std::cout << R"(
[space]  toggle animation
)";

  // -----------------------------

  Eigen::RowVector3f last_mouse;

  const auto &update = [&]()
  {
    if (s.placing_handles)
    {
      viewer.data_list[xid].set_points(s.CV, blue);
      // set new color for the path

      // draw a edge between the last two points
      // if (s.CV.rows() > 1)
      // {
      //   Eigen::MatrixXd edges(2, 3);
      //   edges.row(0) = s.CV.row(s.CV.rows() - 2);
      //   edges.row(1) = s.CV.row(s.CV.rows() - 1);
      //   viewer.data_list[xid].add_edges(edges.row(0), edges.row(1), blue);
      //   // Eigen::MatrixXi EV = Eigen::MatrixXi::Zero(1, 2);
      //   // EV.row(0) << 0, 1;
      //   // viewer.data_list[xid].set_edges(edges, EV, blue);
      // }

    }
    // viewer.data_list[yid].compute_normals();
  };
  viewer.callback_mouse_down =
      [&](igl::opengl::glfw::Viewer &, int, int) -> bool
  {
    // print mouse position
    // double x = viewer.current_mouse_x;
    // double y = viewer.core().viewport(3) - viewer.current_mouse_y;
    // std::cout << "mouse: " << x << ", " << y << std::endl;

    last_mouse = Eigen::RowVector3f(
        viewer.current_mouse_x, viewer.core().viewport(3) - viewer.current_mouse_y, 0);
    if (s.placing_handles)
    {
      // Find closest point on mesh to mouse position
      int fid;
      Eigen::Vector3f bary;
      if (igl::unproject_onto_mesh(
              last_mouse.head(2),
              viewer.core().view,
              viewer.core().proj,
              viewer.core().viewport,
              VX, FX,
              fid, bary))
      {
        long c;
        bary.maxCoeff(&c);
        Eigen::RowVector3d new_c = VX.row(FX(fid, c));
        // append VX.row(FX(fid, c)) to PathIndex
        PathIndex.conservativeResize(PathIndex.rows() + 1);
        PathIndex(PathIndex.rows() - 1) = FX(fid, c);
        if (s.CV.size() == 0 || (s.CV.rowwise() - new_c).rowwise().norm().minCoeff() > 0)
        {
          // push_undo();
          s.CV.conservativeResize(s.CV.rows() + 1, 3);
          // Snap to closest vertex on hit face
          s.CV.row(s.CV.rows() - 1) = new_c;
          update();
          return true;
        }
      }
    }
    return false;
  };
  // -----------------------------

  const auto &single_iteration = [&]()
  {
    // print s.CV
    std::cout << "Path: " << s.CV << std::endl; 
    // print PathIndex
    std::cout << "PathIndex: " << PathIndex << std::endl;

    // // shift x value of s.CV by 1
    // Eigen::MatrixXd CV_shifted = s.CV;
    // CV_shifted.col(0) = CV_shifted.col(0).array() + 0.01;
    // // print CV_shifted
    // std::cout << "CV_shifted: " << CV_shifted << std::endl;
    // s.CV = CV_shifted;

    if (s.CV.rows() > 1){
      // connect the path with edge
      Eigen::MatrixXi EV = Eigen::MatrixXi::Zero(s.CV.rows() - 1, 2);
      for (int i = 0; i < s.CV.rows() - 1; i++)
      {
        EV.row(i) << i, i + 1;
      }
      viewer.data_list[xid].set_edges(s.CV, EV, green);
    }

    if (s.placing_handles != true){
      std::cout << "do single_iteration: --------------------" << std::endl;

      // Eigen::MatrixXi FY;
      // Eigen::MatrixXd VY;
      geodesic_remesh(VX, FX, PathIndex, VY, FY);

      // convert intrinsic edge length to vertices
      // Eigen::MatrixXd lout2 = Eigen::MatrixXd::Zero(VX.rows(), 3);
      // for (int i = 0; i < FY.rows(); i++)
      // {
      //   for (int j = 0; j < 3; j++)
      //   {
      //     lout2.row(FY(i, j)) += VY.row(i);
      //   }
      // }
      // show intrinsic mesh in viewer
      // viewer.data_list[xid].set_mesh(VX, FY);

      // viewer.data_list[yid].set_vertices(lout2);
      // viewer.data_list[yid].set_points(lout2, (1. - (1. - orange.array()) * .8));

      // viewer.data_list[xid].set_edges(VX, FY, Eigen::RowVector3d(0.3, 0.3, 0.3));
      // viewer.data_list[xid].set_colors(VY);
      viewer.data_list[yid].compute_normals();
      // set_points();
    }
    s.placing_handles = false;
  };

  viewer.callback_key_pressed =
      [&](igl::opengl::glfw::Viewer &, unsigned char key, int) -> bool
  {
    switch (key)
    {
    case ' ':
      single_iteration();
      break;
    default:
      return false;
    }
    return true;
  };

  viewer.data_list[xid].set_mesh(VX, FX);
  viewer.data_list[xid].set_colors(orange);
  viewer.data_list[xid].point_size = 10;

  // VX = VX;
  VY = VX;
  FY = FX;

  // Eigen::MatrixXd EmpV;
  // Eigen::MatrixXi EmpF;
  
  viewer.data_list[yid].set_mesh(VY, FY);
  viewer.data_list[yid].set_colors(green);
  // viewer.data_list[yid].line_width = 90;
  // viewer.data().point_size = 10;

  viewer.launch();

  return EXIT_SUCCESS;
}
