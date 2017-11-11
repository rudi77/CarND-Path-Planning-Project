#pragma once
#include <vector>
#include <tuple>
#include "helpers.h"
#include "car.h"

class Map;
class Road;

// BehaviorPlanner for the EgoCar.
// The output of the BehaviorPlanner is a new feasible, safe and efficient trajectory
class BehaviorPlanner
{
public:
  enum State
  {
    KeepLane,
    ChangeLaneLeft,
    ChangeLaneRight,
  };

  explicit BehaviorPlanner(CarState& car) 
    : _car(car), 
      _currentState(KeepLane),
      _target_lane(Center)
  {}

  virtual ~BehaviorPlanner() {}

  std::vector<std::vector<double>> transition(const std::vector<CarState>& other_cars, const Map& map, const Road& road);
  
private:
  CarState& _car;
  State _currentState;
  Lane _target_lane;

  std::vector<std::tuple<State,Lane>> get_possible_states(Lane current_lane) const;
};


