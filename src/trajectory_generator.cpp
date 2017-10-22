#include "trajectory_generator.h"
#include "car.h"
#include "helpers.h"
#include <cassert>
#include "spline.h"
#include <iostream>

using namespace std;

TrajectoryGenerator::TrajectoryGenerator(const Map& map) : _map(map)
{}

vector<vector<double>> TrajectoryGenerator::next_points(const Car& car, const vector<double>& prev_path_x, const vector<double>& prev_path_y)
{
  assert(prev_path_x.size() == prev_path_y.size());

  // Create a list of evenly spaced waypoints.
  vector<double> ptsx;
  vector<double> ptsy;

  auto ref_x = car.x;
  auto ref_y = car.y;
  auto ref_yawn =  deg2rad(car.yaw);

  // Use the car's position as starting point if there is almost any previous points left
  if (prev_path_x.size() < 2)
  {
    // Use two points that makes the path tangent to the car
    auto prev_car_x = ref_x - cos(ref_yawn);
    auto prev_car_y = ref_y - sin(ref_yawn);

    ptsx.push_back(prev_car_x);
    ptsy.push_back(prev_car_y);

    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y);
  }
  // Use previous points as starting points...
  else
  {
    // Redefine reference point as previous path end point
    ref_x = prev_path_x[prev_path_x.size() - 1];
    ref_y = prev_path_y[prev_path_y.size() - 1];

    auto prev_ref_x = prev_path_x[prev_path_x.size() - 2];
    auto prev_ref_y = prev_path_y[prev_path_y.size() - 2];

    ref_yawn = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

    ptsx.push_back(prev_ref_x);
    ptsy.push_back(prev_ref_y);

    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y);
  }

  // Now add evenly spaced points in Frenet coordinates
  auto point1 = _map.get_xy(car.s + 30, 2 + 4 * static_cast<int>(car.current_lane));
  auto point2 = _map.get_xy(car.s + 60, 2 + 4 * static_cast<int>(car.current_lane));
  auto point3 = _map.get_xy(car.s + 90, 2 + 4 * static_cast<int>(car.current_lane));

  ptsx.push_back(point1[0]);
  ptsy.push_back(point1[1]);

  ptsx.push_back(point2[0]);
  ptsy.push_back(point2[1]);
  
  ptsx.push_back(point3[0]);
  ptsy.push_back(point3[1]);

  for (int i = 0; i < ptsx.size(); ++i)
  {
    cout << "Point (" << ptsx[i] << ":" << ptsy[i] << ")" << endl;
  }

  // Shifting anchor points into car reference points
  for (int i = 0; i < ptsx.size(); ++i) {
    auto shift_x = ptsx[i] - ref_x;
    auto shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(0 - ref_yawn) - shift_y * sin(0 - ref_yawn);
    ptsy[i] = shift_x * sin(0 - ref_yawn) + shift_y * cos(0 - ref_yawn);
  }

  // Creating spline through the anchor points
  tk::spline s;
  s.set_points(ptsx, ptsy);

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // Add all points from the previous time
  for (int i = 0; i < prev_path_x.size(); ++i)
  {
    next_x_vals.push_back(prev_path_x[i]);
    next_y_vals.push_back(prev_path_y[i]);
  }

  // Split spline so that we travel at desired velocity
  auto target_x = 30.0;
  auto target_y = s(target_x);
  auto target_dist = distance(target_x, target_y);

  auto ref_vel = 49.5;
  auto x_add_on = 0.0;

  for (auto i = 1; i <= (50 - prev_path_x.size()); ++i)
  {
    auto N = target_dist / (0.02 * ref_vel / 2.24);
    auto x_point = x_add_on + target_x / N;
    auto y_point = s(x_point);

    x_add_on = x_point;

    auto x_ref = x_point;
    auto y_ref = y_point;

    // rotate back
    x_point = x_ref * cos(ref_yawn) - y_ref * sin(ref_yawn);
    y_point = x_ref * sin(ref_yawn) + y_ref * cos(ref_yawn);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  assert(next_x_vals.size() == next_y_vals.size());
  assert(next_x_vals.size() == 50);

  return { next_x_vals, next_y_vals };
}