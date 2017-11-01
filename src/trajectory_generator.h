#pragma once
#include <vector>
#include "map.h"

class Car;

class TrajectoryGenerator
{
private:
  Map _map;

public:
  explicit TrajectoryGenerator(const Map& map);

  std::vector<Car> compute_trajectory(const Car& car);
  //std::vector<std::vector<double>> compute_trajectory(const Car& car);
};