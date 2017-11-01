#pragma once
#include <map>
#include <vector>
#include "car.h"

class Map;

// The Predictor can be used to predict the future trajectories
// of detected vehicles coming from the sensor fusion module.
class Predictor
{
public:
  // Predicts the trajectories of other cars.
  static std::map<int, std::vector<Car>> predict_trajectories(const std::vector<Car>& other_cars, const Map& map);
};
