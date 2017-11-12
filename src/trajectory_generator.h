#pragma once
#include <vector>
#include "map.h"
#include "helpers.h"

class CarState;

class TrajectoryGenerator
{
public:
  explicit TrajectoryGenerator(const Map& map);

  std::vector<CarState> compute_trajectory(const CarState& car, double target_speed, Lane target_lane);

private:
  Map _map;

  static std::vector<double> perturb_data(std::vector<double> means, std::vector<double> sigmas);
};