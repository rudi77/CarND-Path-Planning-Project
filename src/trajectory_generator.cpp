#include "trajectory_generator.h"
#include "car.h"
#include "helpers.h"
#include <cassert>
#include "spline.h"
#include <iostream>

using namespace std;

TrajectoryGenerator::TrajectoryGenerator(const Map& map) : _map(map)
{}

vector<Car> TrajectoryGenerator::compute_trajectory(const Car& car)
//vector<vector<double>> TrajectoryGenerator::compute_trajectory(const Car& car)
{
  // Create a list of evenly spaced waypoints.
  vector<double> anchor_points_x;
  vector<double> anchor_points_y;

  // Current position of the car in global coordinates
  auto ref_car_x    = car.x;
  auto ref_car_y    = car.y;
  auto ref_car_yaw  = deg2rad(car.yaw);

  // Use the car's position as starting point if there is almost any previous points left
  if (car.prev_waypoints.size() < 2)
  {
    // Use two points that makes the path tangent to the car
    auto prev_car_x = ref_car_x - cos(ref_car_yaw);
    auto prev_car_y = ref_car_y - sin(ref_car_yaw);

    anchor_points_x.push_back(prev_car_x);
    anchor_points_y.push_back(prev_car_y);

    anchor_points_x.push_back(ref_car_x);
    anchor_points_y.push_back(ref_car_y);
  }
  // Use previous points as starting points...
  else
  {
    // Redefine reference point as previous path end point
    ref_car_x = car.prev_waypoints[car.prev_waypoints.size() - 1].x;
    ref_car_y = car.prev_waypoints[car.prev_waypoints.size() - 1].y;

    auto prev_ref_x = car.prev_waypoints[car.prev_waypoints.size() - 2].x;
    auto prev_ref_y = car.prev_waypoints[car.prev_waypoints.size() - 2].y;

    ref_car_yaw = atan2(ref_car_y - prev_ref_y, ref_car_x - prev_ref_x);

    anchor_points_x.push_back(prev_ref_x);
    anchor_points_y.push_back(prev_ref_y);

    anchor_points_x.push_back(ref_car_x);
    anchor_points_y.push_back(ref_car_y);
  }

  // Now add evenly spaced anchor points in Frenet coordinates
  auto start_s = 30;
  for (auto i = start_s; i <= 90; i += 30)
  {
    auto anchor_point = _map.get_xy(car.s + i, 2 + 4 * static_cast<int>(car.target_lane));
    anchor_points_x.push_back(anchor_point[0]);
    anchor_points_y.push_back(anchor_point[1]);
  }

  // Shifting anchor points into car reference points
  for (int i = 0; i < anchor_points_x.size(); ++i) {
    auto shift_x = anchor_points_x[i] - ref_car_x;
    auto shift_y = anchor_points_y[i] - ref_car_y;

    anchor_points_x[i] = shift_x * cos(0 - ref_car_yaw) - shift_y * sin(0 - ref_car_yaw);
    anchor_points_y[i] = shift_x * sin(0 - ref_car_yaw) + shift_y * cos(0 - ref_car_yaw);
  }

  // Creating spline through the anchor points
  tk::spline s;
  s.set_points(anchor_points_x, anchor_points_y);

  vector<Car> trajectory;
  //vector<vector<double>> trajectory;

  // Add all points from the previous time
  for (int i = 0; i < car.prev_waypoints.size(); ++i)
  {
    auto x_point = car.prev_waypoints[i].x;
    auto y_point = car.prev_waypoints[i].y;
    auto s_d_values = _map.get_frenet(x_point, y_point, ref_car_yaw);

    Car next_car_state(car.id, x_point, y_point, s_d_values[0], s_d_values[1], car.speed, rad2deg(ref_car_yaw));
    trajectory.push_back(next_car_state);
    //trajectory.push_back({ x_point, y_point, s_d_values[0], s_d_values[1], car.speed, rad2deg(ref_car_yaw) });
  }

  // Split spline so that we travel at desired velocity
  auto target_x = 30.0;
  auto target_y = s(target_x);
  auto target_dist = distance(target_x, target_y);

  // starting point in our car's locale coordinate system
  auto x_add_on = 0.0;
  auto current_speed = car.speed;
  auto acc = AccMax - 2.0; // m/s^2

  for (auto i = 1; i <= 50 - car.prev_waypoints.size(); ++i)
  {
    auto x_point = 0.0;
    auto y_point = 0.0;

    // stop accelerating
    if (current_speed == car.target_speed)
    {
      acc = 0.0;
    }
    else if (current_speed > car.target_speed)
    {
      acc = -1.0 * acc;
    }
    else
    {
      acc = abs(acc);
    }
    
    x_point = pos_new(x_add_on, current_speed / 2.24, acc);
    current_speed = v_new(current_speed, acc);


    //auto N = target_dist / (0.02 * current_speed / 2.24); // divide by 2.24 because we need this in m/s

    //// Calc the next point in the car's local coordinate system
    //auto x_point = x_add_on + target_x / N;

    y_point = s(x_point);

    x_add_on = x_point;

    auto x_ref = x_point;
    auto y_ref = y_point;

    // transform coordinates back to global coordinate system
    x_point = x_ref * cos(ref_car_yaw) - y_ref * sin(ref_car_yaw);
    y_point = x_ref * sin(ref_car_yaw) + y_ref * cos(ref_car_yaw);

    x_point += ref_car_x;
    y_point += ref_car_y;

    auto s_d_values = _map.get_frenet(x_point, y_point, ref_car_yaw);

    Car next_car_state(car.id, x_point, y_point, s_d_values[0], s_d_values[1], current_speed, rad2deg(ref_car_yaw));        
    trajectory.push_back(next_car_state);

    //trajectory.push_back({ x_point, y_point, s_d_values[0], s_d_values[1], current_speed, rad2deg(ref_car_yaw) });
  }

  assert(trajectory.size() == 50);

  return trajectory;
}