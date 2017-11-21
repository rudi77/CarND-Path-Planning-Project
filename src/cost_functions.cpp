#include "cost_functions.h"
#include "road.h"

double speed_cost(const double speed)
{
  auto target_v = OptimalSpeed;
  auto k = 1.0 / target_v;
  auto d = k * target_v;

  if (speed < target_v)
  {
    return (-1.0 / target_v) * speed + 1.0;
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

double collision_cost(int prev_waypoints, const std::vector<CarState>& trajectory, const std::vector<CarState>& other_cars)
{
  // Checks whether there is possible collision with any car along
  // the provided trajectory.
  auto N = trajectory.size();

  for (auto carState : other_cars)
  {
    for (unsigned int i = prev_waypoints; i <= N; i++)
    {
      auto isCollision = trajectory[i].collides_with(carState, prev_waypoints*DeltaT + i*DeltaT);
      if (isCollision)
      {
        std::cout << "COLLSION COST END WITH COLLISION" << std::endl;
        return 1.0;
      }
    }
  }

  return 0.0;
}

double free_lane_cost(const CarState& car, const Road& road, Lane target_lane)
{
  auto nearest_leading = road.nearest_leading_in_lane(target_lane);
  return (nearest_leading != nullptr && nearest_leading->is_too_close(car, 50)) ? 1.0 : 0.0;
}

double not_in_center_lane_cost(Lane target_lane)
{
  return target_lane == Center ? 0.0 : 1.0;
}

double Costs::calc_cost(
  const CarState& egocar,
  const std::vector<CarState>& trajectory,
  const Road& road,
  const double target_speed,
  const Lane current_lane,
  const Lane target_lane,
  const int prev_waypoints)
{
  auto collision_cost   = ::collision_cost(prev_waypoints * DeltaT, trajectory, road.other_cars()) * 1000;
  auto speed_cost       = ::speed_cost(target_speed) * 100;
  auto lane_change_cost = ::lane_change_cost(current_lane, target_lane) * 50;
  auto free_lane_cost   = ::free_lane_cost(egocar, road, target_lane);
  auto center_lane_cost = ::not_in_center_lane_cost(target_lane) * 5;

  return collision_cost + speed_cost + lane_change_cost + free_lane_cost + center_lane_cost;
}