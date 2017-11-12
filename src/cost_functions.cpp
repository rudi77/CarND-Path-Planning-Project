#include "cost_functions.h"

double speed_cost(const double speed)
{
  auto target_v = OptimalSpeed;
  auto k = 1.0 / target_v;
  auto d = k * target_v;

  if (speed < target_v)
  {
    return (-0.8 / target_v) * speed + 0.8;
  }

  if (speed == target_v)
  {
    return 0.0;
  }

  if (target_v < speed && speed <= SpeedLimit)
  {
    return (-1.0 / target_v) * speed + 1.0;
  }
    
  return 1.0;
}

double lane_change_cost(Lane current_lane, Lane target_lane)
{
  return current_lane == target_lane ? 0.0 : 1.0;
}

double Costs::calc_cost(
  const std::vector<CarState>& trajectory,
  const std::vector<CarState>& other_cars,
  const double target_speed,
  const Lane current_lane,
  const Lane target_lane)
{
  auto speed_cost = ::speed_cost(target_speed) * 100;
  auto lane_change_cost = ::lane_change_cost(current_lane, target_lane) * 10;

  //std::cout << "TargetLane " << target_lane << " CurrentLane " << current_lane << " TargetSpeed " << target_speed 
  //          << " LaneChangeCost " << lane_change_cost << " SpeedCost " << speed_cost << " CostTotal " 
  //          << speed_cost + lane_change_cost << std::endl;

  return speed_cost + lane_change_cost;
}