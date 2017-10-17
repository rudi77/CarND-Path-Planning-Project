#include "trajectory_generator.h"

TrajectoryGenerator::TrajectoryGenerator(const Map& map) : _map(map)
{}

std::vector<std::vector<double>> TrajectoryGenerator::next_points(double car_s)
{
  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  // We will set the points 0.5 m apart. Since the car moves 50 times a second, 
  // a distance of 0.5m per move will create a velocity of 25 m/s. 25 m/s is close to 50 MPH.
  auto dist_inc = 0.5;

  for (auto i = 0; i < 50; i++)
  {
    // i+1 because this is the next point 
    auto next_s = car_s + (i + 1)*dist_inc;

    // one and half lanes away from the waypoint -> second lane!
    auto next_d = 6.0;

    auto xy = _map.get_xy(next_s, next_d);

    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);
  }

  return { next_x_vals, next_y_vals };
}