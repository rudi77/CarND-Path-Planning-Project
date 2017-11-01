#pragma once
#include <vector>
#include "car.h"
#include <map>

class Map;

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

  explicit BehaviorPlanner(Car& car, double target_s) 
    : car_(car), target_s_(target_s) 
  {}

  virtual ~BehaviorPlanner() {}

  std::vector<std::vector<double>> transition(const std::map<int, std::vector<Car>>& other_cars, const Map& map);
  //std::vector<std::vector<double>> transition(const std::map<int, std::vector<std::vector<double>>>& other_cars, const Map& map);


private:
  Car& car_;
  State _currentState = KeepLane; // Default state.

  double target_s_;

  std::vector<State> get_possible_states(Lane current_lane) const;

  // Calculates a cost value for a certain trajectory
  double calc_cost(const std::vector<Car> ego_car_trajetory, const std::map<int, std::vector<Car>>& other_cars_trajectories);

  static double speed_cost(const double speed);

  double distance_cost();


  // Gets the car which is in the same lane and has minimum distance to the ego car.
  int get_nearest_leading(const std::map<int, std::vector<Car>>& other_cars, int current_lane) const;
};


