#pragma once
#include <vector>
#include "map.h"
//#include "car.h"

class Car;

class TrajectoryGenerator
{
private:
  Map _map;

public:
  TrajectoryGenerator(const Map& map);

  std::vector<std::vector<double>> next_points(const Car& car , const std::vector<double>& prev_path_x, const std::vector<double>& prev_path_y);
};