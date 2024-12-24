#include <fstream>
#include <iostream>
#include <memory>
#include "src/reeds_shepp_path.h"
#include "src/basic_type.h"
#include "src/hybrid_a_star.h"
#include "result_plot/cpp_plot.h"

namespace plt = matplotlibcpp;

void printHybridAStarResult(const HybridAStartResult& result, const std::string& file_name) {
    //std::ofstream file(file_name);
    std::ofstream file(file_name);
    if (file.is_open()) {
        for (size_t i = 0; i < result.x.size(); ++i) {
            file << result.x[i] << " " << result.y[i] << " " << result.phi[i] << " " << result.v[i] << std::endl;
        }
        file.close();
        std::cout << "Successfully printed HybridAStarResult to " << file_name << std::endl;
    } else {
        std::cout << "Error opening file " << file_name << std::endl;
    }
}
int main() {

  ParkingScenario parking_scenario;
  // parking_scenario.start_pos = {-1.5, -3.72, 1.57};// {-1.634, 0.634, 1.8}
  // parking_scenario.end_pos = {-1.634, 0.634, 1.8};  // {-1.5, -3.72, 1.57}
  // parking_scenario.start_pos = {-0.726, -3.641, 1.8};// {-1.634, 0.634, 1.8}
  // parking_scenario.end_pos = {-1.634, 0.634, 1.8};  // {-1.5, -3.72, 1.57}

  parking_scenario.start_pos = {-10.50, -0.720, 1.507};// {-1.634, 0.634, 1.8}
  parking_scenario.end_pos = {-2.007, 1.255, 1.953};  // {-1.5, -3.72, 1.57}
  // parking_scenario.start_pos = {-0.105, -3.451, 1.953};// {-1.634, 0.634, 1.8}
  // parking_scenario.end_pos = {-2.007, 1.255, 1.953};  // {-1.5, -3.72, 1.57}

  parking_scenario.boundary = {-13.00, 10.00, -8.00,10.00};
  parking_scenario.obstacles = {{{-9.97, -0.08}, {-4.34, -0.08}, {-4.34, -6.01}, {1.00, -6.05}, {1.13, -0.04}, {6.97, -0.04}, {6.97, 6.91}, {-9.97, 7.04}, {-9.89, -0.12}}};

  WarmStartConfig warm_start_config;
  warm_start_config.xy_grid_resolution = 0.2;
  warm_start_config.phi_grid_resolution = 0.0;
  warm_start_config.next_node_num = 10;
  warm_start_config.step_size = 0.2;
  warm_start_config.traj_forward_penalty = 10.0;
  warm_start_config.traj_back_penalty = 0.0;
  warm_start_config.traj_gear_switch_penalty = 100.0;
  warm_start_config.traj_steer_penalty = 10.0;
  warm_start_config.traj_steer_change_penalty = 10.0;
  warm_start_config.grid_a_star_xy_resolution = 0.1;
  warm_start_config.node_radius = 0.5;
  warm_start_config.delta_t = 1.0;

  VehicleParam vehicle_param;
  vehicle_param.front_edge_to_center = 3.89;
  vehicle_param.back_edge_to_center = 1.043;
  vehicle_param.length = 4.933;
  vehicle_param.width = 2.11;
  vehicle_param.max_steer_angle = 7.0;
  vehicle_param.steer_ratio = 16.0;
  vehicle_param.wheel_base = 2.8448;

//******************************************************************************
  plt::figure();
  plt::xlim(parking_scenario.boundary[0], parking_scenario.boundary[1]);
  plt::ylim(parking_scenario.boundary[2], parking_scenario.boundary[3]);
  plt::set_aspect(1);
  plt::grid(true);
  plt::title("Hybrid_Astar_Parking");
//*****************************************************************************
}