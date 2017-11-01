#pragma once
#include <vector>
#include "map.h"
#include "helpers.h"

class Car;

class TrajectoryGenerator
{
private:
  Map _map;

public:
  explicit TrajectoryGenerator(const Map& map);

  std::vector<Car> compute_trajectory(Car& car, double target_speed, Lane target_lane);
};