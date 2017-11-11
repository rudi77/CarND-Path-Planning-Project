#pragma once

#include <vector>
#include "car.h"

class Costs
{
public:
  static double calc_cost(
    const std::vector<CarState>& trajectory, 
    const std::vector<CarState>& other_cars,
    const double target_speed,
    const Lane current_lane,
    const Lane target_lane
  );
};