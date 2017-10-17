#pragma once
#include <vector>
#include "map.h"

class TrajectoryGenerator
{
private:
  Map _map;

public:
  TrajectoryGenerator(const Map& map);

  std::vector<std::vector<double>> next_points(double car_s);
};