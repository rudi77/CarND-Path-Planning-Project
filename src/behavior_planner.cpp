#include "behavior_planner.h"
#include "map.h"
#include "car.h"
#include "trajectory_generator.h"

double BehaviorPlanner::speed_cost(const double speed)
{
  auto target_v = SpeedLimit - SpeedBuffer;
  auto k = 1.0 / target_v;
  auto d = k * target_v;

  if (speed < target_v)
    return (-0.8 / target_v) * speed + 0.8;

  if (speed == target_v)
    return 0.0;

  if (target_v < speed && speed <= SpeedLimit)
    return k * speed + d;

  return 1.0;
}

double BehaviorPlanner::distance_cost()
{
}

std::vector<std::vector<double>> BehaviorPlanner::transition(const std::map<int, std::vector<Car>>& other_cars, const Map& map)
{
  TrajectoryGenerator generator(map);

  // Get all possible states based on the current state and current line
  auto current_lane = car_.current_lane;
  auto possible_states = get_possible_states(current_lane);

  std::map<State, std::vector<Car>> possible_trajectories;
  std::map<State, double> possible_costs;

  std::vector<std::vector<double>> next_waypoints;

  // For each possible state calculate a new possible trajectory
  // NOTE: possible trajectories will not take any previous waypoints into account
  // whereas the final trajectory will take.
  for (auto possible_state : possible_states)
  { 
    switch (possible_state)
    {
      case KeepLane: 
      { 
        auto target_lane = current_lane;

        //get nearest leading car in the same line and check the distance to the ego car at the last time step
        auto nearest_leading_car = get_nearest_leading(other_cars, current_lane);

        std::vector<Car> kl_trajectory;

        auto target_speed = 0.0;

        if (nearest_leading_car != -1 &&  other_cars.at(nearest_leading_car)[0].is_too_close(car_))
        {
          auto clone = car_.clone();

          //std::cout << "CurrentSpeed " << clone.speed << " setting target speed from " << clone.target_speed << " to " << other_cars.at(nearest_leading_car)[0].speed << std::endl;

          target_speed = other_cars.at(nearest_leading_car)[0].speed;

          kl_trajectory = generator.compute_trajectory(clone, target_speed, target_lane);
        }
        else
        {
          target_speed = RefSpeed;
          kl_trajectory = generator.compute_trajectory(car_, target_speed, target_lane);
        }

        possible_trajectories[KeepLane] = kl_trajectory;
        possible_costs[KeepLane] = speed_cost(target_speed);
        
        break;
      }
      case ChangeLaneLeft:
      {
        auto left_lane = get_lane_left(current_lane);
        auto clf_trajectory = generator.compute_trajectory(car_, RefSpeed, left_lane);
        possible_trajectories[ChangeLaneLeft] = clf_trajectory;
        break;
      }      
      case ChangeLaneRight: 
      {       
        auto right_lane = get_lane_right(current_lane);
        auto clf_trajectory = generator.compute_trajectory(car_, RefSpeed, right_lane);
        possible_trajectories[ChangeLaneRight] = clf_trajectory;
        break;
      }
    }
    // For each trajectory compute a cost

    // Take the trajectory with min cost.
  }

  for (auto t : possible_trajectories[KeepLane])
  {
    next_waypoints.push_back({ t.x, t.y });
  }

  assert(next_waypoints.size() == 50);

  return next_waypoints;
}

std::vector<BehaviorPlanner::State> BehaviorPlanner::get_possible_states(Lane current_lane) const
{
  std::vector<State> possible_states;

  switch (_currentState)
  {
    case State::KeepLane:
    {
      if (current_lane == Left)
      {
        possible_states = { State::KeepLane, State::ChangeLaneRight };
      }
      if (current_lane == Right)
      {
        possible_states = { State::KeepLane, State::ChangeLaneLeft };
      }

      possible_states = { State::KeepLane, State::ChangeLaneLeft, State::ChangeLaneRight };

      break;
    }
    case ChangeLaneLeft:
    {
      if (current_lane == Left)
      {
        possible_states = { State::KeepLane };
      }

      possible_states = { State::KeepLane, State::ChangeLaneLeft };

      break;
    }
    case ChangeLaneRight:
    {
      if (current_lane == Right)
      {
        possible_states = { State::KeepLane };
      }

      possible_states = { State::KeepLane, State::ChangeLaneRight };
      break;
    }
  }

  return possible_states;
}

int BehaviorPlanner::get_nearest_leading(const std::map<int, std::vector<Car>>& other_cars_map, int current_lane) const
{
  std::vector<int> vehicles_in_lane;

  // pointer to the nearest car in front of the EGOCAR
  auto car_in_front = -1;

  // Iterate over each car and check if it is in the same lane
  // and in front of the ego car. And then check whether it is
  // the nearest leading car.
  for (auto pair : other_cars_map)
  {
    auto other_car = pair.second[0];

    if (other_car.is_in_lane(car_) && other_car.is_in_front(car_))
    {
      if (car_in_front == -1)
      {
        car_in_front = other_car.id;
      }
      else if (other_car.s < other_cars_map.at(car_in_front)[0].s)
      {
        car_in_front = other_car.id;
      }
    }
  }

  return car_in_front;
}

double BehaviorPlanner::calc_cost(const std::vector<Car> ego_car_trajetory, const std::map<int, std::vector<Car>>& other_cars_trajectories)
{

}
