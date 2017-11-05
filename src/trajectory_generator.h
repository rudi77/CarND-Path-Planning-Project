#pragma once
#include <vector>
#include "map.h"
#include "jmt.h"
#include "helpers.h"

class CarState;

class TrajectoryGenerator
{
public:
  explicit TrajectoryGenerator(const Map& map);

  std::vector<CarState> compute_trajectory(CarState& car, double target_speed, Lane target_lane);

private:
  Map _map;

  std::vector<CarState> keep_lane_trajectory(
    CarState& car,
    std::vector<double> anchor_points_x,
    std::vector<double> anchor_points_y,
    double target_speed,
    double ref_car_x,
    double ref_car_y,
    double ref_car_yaw);

  std::vector<std::vector<double>> switch_lane_trajectory(
    std::vector<double> s_start,
    std::vector<double> s_end,
    std::vector<double> d_start,
    std::vector<double> d_end,
    double t
  );
};