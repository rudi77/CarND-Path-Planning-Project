#include <map>
#include "map.h"
#include "predictor.h"
#include "trajectory_generator.h"

using namespace std;

map<int, vector<CarState>> Predictor::predict_trajectories(const vector<CarState>& other_cars, const Map& map)
{
  TrajectoryGenerator trajectory_generator(map);

  std::map<int, vector<CarState>> trajectories;
  for (auto car : other_cars)
  {
    auto trajectory = trajectory_generator.compute_trajectory(car, car.speed, car.current_lane);

    assert(trajectory.size() == 50);

    trajectory.insert(trajectory.begin(), car);
    trajectories[car.id] = trajectory;
  }

  return trajectories;
}
