#include <iostream>

#include "behavior_planner.h"
#include "map.h"
#include <iostream>
#include <string>

#include "road.h"
#include "car.h"
#include "trajectory_generator.h"
#include "cost_functions.h"
#include <algorithm>

std::string state_name(BehaviorPlanner::State state)
{
  switch (state)
  {
  case BehaviorPlanner::KeepLane: return "KeepLane";
  case BehaviorPlanner::ChangeLaneLeft: return "ChangeLaneLeft";
  case BehaviorPlanner::ChangeLaneRight: return "ChangeLaneRight";
  default: return "UNKOWN";
  }
}

void print_states_vs_costs(
  const std::vector<std::tuple<BehaviorPlanner::State, Lane>>& states, 
  std::vector<std::tuple<double, Lane, Lane, double, double>> costs_,
  const int bestCostIdx)
{
  assert(states.size() == costs_.size());

  std::cout << "State" << "\t\t" "CurrentLane" << "\t\t" << "TargetLane" << "\t\t" "CurrentSpeed" << "\t\t" << "TargetSpeed" << "\t\t" << "Costs" << "\t\t" << "BestCost" <<"\n";
  std::cout << "========================================================================================================================================\n";

  for (unsigned int i = 0; i < states.size(); ++i)
  {
    auto state          = std::get<0>(states[i]);

    auto cost           = std::get<0>(costs_[i]);
    auto current_lane   = std::get<1>(costs_[i]);
    auto target_lane    = std::get<2>(costs_[i]);
    auto current_speed  = std::get<3>(costs_[i]);
    auto target_speed   = std::get<4>(costs_[i]);

    std::cout << state_name(state)  
              << "\t\t" << current_lane << "\t\t" << target_lane 
              << "\t\t\t" << current_speed << "\t\t\t" << target_speed 
              << "\t\t\t" << round2(cost, 2)
              << "\t\t" << (bestCostIdx == i ? "Best" : "Rest") << "\n";
  }

  std::cout << std::endl;
}

std::vector<std::vector<double>> BehaviorPlanner::transition(const std::vector<CarState>& other_cars, const Map& map, const Road& road)
{
  TrajectoryGenerator generator(map);

  // Get all possible states based on the current state and current line
  auto possible_states = get_possible_states(_car.current_lane);

  std::vector<std::vector<CarState>> possible_trajectories;
  std::vector<std::tuple<double, Lane, Lane, double, double>> costs_;
  std::vector<double> costs;

  // Calculate one trajectory for each possible state
  for (auto possible_state_tuple : possible_states)
  {
    auto state = std::get<0>(possible_state_tuple);
    auto target_lane = std::get<1>(possible_state_tuple);
    auto target_speed = OptimalSpeed;

    // Check if there is a leading car on the target lane.
    // Check if it is too close.
    // If too close reduce speed
    auto leading_car_id = road.nearest_leading_in_lane(target_lane);

    if (leading_car_id != -1)
    {
      CarState leading_car(-1);

      for (unsigned int i = 0; i < other_cars.size(); ++i)
      {
        if (other_cars[i].id == leading_car_id)
        {
          leading_car = other_cars[i];
          break;
        }
      }

      if (leading_car.is_too_close(_car))
      {
        target_speed = leading_car.speed;
      }
    }

    auto possible_trajectory = generator.compute_trajectory(_car, target_speed, target_lane);
    auto cost_total = Costs::calc_cost(possible_trajectory, other_cars, target_speed, _car.current_lane, target_lane);

    possible_trajectories.push_back(possible_trajectory);
    costs.push_back( cost_total );

    costs_.push_back(std::make_tuple(cost_total, _car.current_lane, target_lane, _car.speed, target_speed));
  }

  // get argmin
  auto min_idx = std::distance(costs.begin(), min_element(costs.begin(), costs.end()));
  auto best_trajectory = possible_trajectories[min_idx];

  print_states_vs_costs(possible_states, costs_, min_idx);

  _currentState = std::get<0>(possible_states[min_idx]);
  _car.speed_prev = best_trajectory[best_trajectory.size() - 1].speed;

  // Set new state and return the trajectory with min(cost) 
  return to_xy_vectors(best_trajectory);
}

std::vector<std::tuple<BehaviorPlanner::State,Lane>> BehaviorPlanner::get_possible_states(Lane current_lane) const
{
  std::vector<std::tuple<State, Lane>> possible_states;

  switch (_currentState)
  {
    case KeepLane:
    {
      if (current_lane == Left)
      {
        possible_states = 
        { 
          std::make_tuple(KeepLane, current_lane), 
          std::make_tuple(ChangeLaneRight, Center) 
        };
      }

      if (current_lane == Center)
      {
        possible_states =
        {
          std::make_tuple(ChangeLaneLeft, Left),
          std::make_tuple(KeepLane, current_lane),
          std::make_tuple(ChangeLaneRight, Right)
        };
      }

      if (current_lane == Right)
      {
        possible_states = 
        { 
          std::make_tuple(ChangeLaneLeft, Center),
          std::make_tuple(KeepLane, current_lane)
        };
      }
      
      break;
    }
    case ChangeLaneLeft:
    {
      if (current_lane == Left)
      {
        possible_states = { std::make_tuple(KeepLane, current_lane) };
      }

      if (current_lane == Center)
      {
        possible_states = { std::make_tuple(ChangeLaneLeft, Left) };
      }

      if (current_lane == Right)
      {
        possible_states = { std::make_tuple(ChangeLaneLeft, Center) };
      }

      break;
    }
    case ChangeLaneRight:
    {
      if (current_lane == Left)
      {
        possible_states = { std::make_tuple(ChangeLaneRight, Center) };
      }

      if (current_lane == Center)
      {
        possible_states = { std::make_tuple(ChangeLaneRight, Right) };
      }

      if (current_lane == Right)
      {
        possible_states = { std::make_tuple(KeepLane, current_lane) };
      }

      break;
    }
  }

  return possible_states;
}
