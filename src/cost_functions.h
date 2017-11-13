#pragma once

#include <vector>
#include "car.h"

class Road;

class Costs
{
public:
  static double calc_cost(
    const CarState& egocar,
    const std::vector<CarState>& trajectory, 
    const Road& road,
    const double target_speed,
    const Lane current_lane,
    const Lane target_lane,
    const int prev_waypoints
  );
};