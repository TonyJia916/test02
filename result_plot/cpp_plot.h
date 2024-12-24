#ifndef CPP_PLOT
#define CPP_PLOT

#include <cmath>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

void PlotVehicle(const VehicleParam &vehicle_param, double x, double y,
                 double phi);



void PlotObstacles_Init(std::vector<std::vector<Vec2d>> obstacles_vertices_vec) {
  if (obstacles_vertices_vec.empty()) {
    return;
  }

  for (const auto &obstacles_vertices : obstacles_vertices_vec) {
    size_t vertices_num = obstacles_vertices.size();
    std::vector<double> obstacles_x;
    std::vector<double> obstacles_y;
    for (size_t i = 0; i <= vertices_num-2; ++i) {
      obstacles_x.emplace_back(obstacles_vertices[i].x);
      obstacles_y.emplace_back(obstacles_vertices[i].y);
      obstacles_x.emplace_back(obstacles_vertices[i + 1].x);
      obstacles_y.emplace_back(obstacles_vertices[i + 1].y);
      
      plt::plot(obstacles_x, obstacles_y, "b-o");
      plt::pause(0.2);
      } 
  }
}


void PlotObstacles(std::vector<std::vector<Vec2d>> obstacles_vertices_vec) {
  if (obstacles_vertices_vec.empty()) {
    return;
  }

  for (const auto &obstacles_vertices : obstacles_vertices_vec) {
    size_t vertices_num = obstacles_vertices.size();
    std::vector<double> obstacles_x;
    std::vector<double> obstacles_y;
    for (size_t i = 0; i <= vertices_num-2; ++i) {
      obstacles_x.emplace_back(obstacles_vertices[i].x);
      obstacles_y.emplace_back(obstacles_vertices[i].y);
      obstacles_x.emplace_back(obstacles_vertices[i + 1].x);
      obstacles_y.emplace_back(obstacles_vertices[i + 1].y);
      
      plt::plot(obstacles_x, obstacles_y, "blue");
      //plt::pause(0.1);
      } 
  }
}

void PlotTrajectory(const std::vector<double> &x,
                    const std::vector<double> &y) {
  plt::plot(x, y, "red");
  plt::axis("equal");
}

void PlotVehicleTraj(const VehicleParam &vehicle_param,
                     const std::vector<double> &traj_x,
                     const std::vector<double> &traj_y,
                     const std::vector<double> &traj_phi,
                     std::vector<std::vector<Vec2d>> obstacles_vertices_vec,
                     const std::vector<double>XYbounds) {
  //PlotTrajectory(traj_x, traj_y);  
  //plt::pause(0.05);              
  double CX = 0;
  double CY = 0;
  double Cphi = 0;
  
  for (size_t i = 0; i < traj_x.size(); ++i) {
    //PlotVehicle(vehicle_param, traj_x[i], traj_y[i], traj_phi[i]);
    CX = traj_x[i];
    CY = traj_y[i];
    Cphi = traj_phi[i];
    PlotVehicle(vehicle_param, CX, CY, Cphi);
    PlotObstacles(obstacles_vertices_vec);
    PlotTrajectory(traj_x, traj_y);
    plt::xlim(XYbounds[0],XYbounds[1]); 
    plt::ylim(XYbounds[2],XYbounds[3]);
    plt::grid(true);
    plt::title("Hybrid_Astar_Parking"); 
    plt::pause(0.1);
    //plt::cla();
  }
 
  //PlotTrajectory(traj_x, traj_y);  
  
}


void PlotVehicle(const VehicleParam &vehicle_param, double x, double y,
                 double phi) {
  const double cos_heading = std::cos(phi);
  const double sin_heading = std::sin(phi);

  // front dx dy
  const double front_dx = cos_heading * vehicle_param.front_edge_to_center;
  const double front_dy = sin_heading * vehicle_param.front_edge_to_center;

  // back dx dy
  const double back_dx = cos_heading * vehicle_param.back_edge_to_center;
  const double back_dy = sin_heading * vehicle_param.back_edge_to_center;

  // width dx dy
  const double dx2 = sin_heading * vehicle_param.width / 2.0;
  const double dy2 = -cos_heading * vehicle_param.width / 2.0;

  std::vector<Vec2d> corners;
  corners.emplace_back(Vec2d{x + front_dx + dx2, y + front_dy + dy2});
  corners.emplace_back(Vec2d{x + front_dx - dx2, y + front_dy - dy2});
  corners.emplace_back(Vec2d{x - back_dx - dx2, y - back_dy - dy2});
  corners.emplace_back(Vec2d{x - back_dx + dx2, y - back_dy + dy2});

  std::vector<double> vehicle_corner_x, vehicle_corner_y;
  for (const auto &coner : corners) {
    vehicle_corner_x.emplace_back(coner.x);
    vehicle_corner_y.emplace_back(coner.y);
  }
  vehicle_corner_x.emplace_back(corners.front().x);
  vehicle_corner_y.emplace_back(corners.front().y);

  plt::plot(vehicle_corner_x, vehicle_corner_y, "green");
  //plt::axis("equal");
  //plt::pause(0.2);
}

#endif
