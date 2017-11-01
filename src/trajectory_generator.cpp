#include "trajectory_generator.h"
#include "car.h"
#include "helpers.h"
#include <cassert>
#include "spline.h"
#include <iostream>

using namespace std;

TrajectoryGenerator::TrajectoryGenerator(const Map& map) : _map(map)
{}

vector<Car> TrajectoryGenerator::compute_trajectory(Car& car, double target_speed, Lane target_lane)
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
    auto anchor_point = (i == 30) 
      ? _map.get_xy(car.s + i, 2 + 4 * static_cast<int>(target_lane))
      : _map.get_xy(car.s + i, 2 + 4 * static_cast<int>(target_lane));

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

  // Add all points from the previous time
  for (int i = 0; i < car.prev_waypoints.size(); ++i)
  {
    auto x_point = car.prev_waypoints[i].x;
    auto y_point = car.prev_waypoints[i].y;
    auto s_d_values = _map.get_frenet(x_point, y_point, ref_car_yaw);

    Car next_car_state(car.id, x_point, y_point, s_d_values[0], s_d_values[1], car.speed, rad2deg(ref_car_yaw));
    trajectory.push_back(next_car_state);
  }

  // Split spline so that we travel at desired velocity
  auto target_x = 30.0;
  auto target_y = s(target_x);
  auto target_dist = distance(target_x, target_y);

  // starting point in our car's locale coordinate system
  auto x_add_on = 0.0;
  auto a = 3;
  auto current_speed = car.speed_prev;

  //std::cout << "currentspeed " << current_speed << std::endl;
  
  for (auto i = 1; i <= 50 - car.prev_waypoints.size(); ++i)
  {
    if (current_speed < target_speed)
    {
      a = 3;
    }
    else if (current_speed > target_speed)
    {
      a = -3;
    }

    auto x_point = pos_new(x_add_on, current_speed / Ms2Mps, a);
    auto y_point = s(x_point);

    current_speed = v_new(current_speed / Ms2Mps, a) * Ms2Mps;

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
  }

  car.speed_prev = current_speed;

  assert(trajectory.size() == 50);

  return trajectory;
}