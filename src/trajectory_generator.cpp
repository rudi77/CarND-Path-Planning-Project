#include "trajectory_generator.h"
#include "car.h"
#include "helpers.h"
#include <cassert>
#include "spline.h"
#include <iostream>

using namespace std;

TrajectoryGenerator::TrajectoryGenerator(const Map& map) : _map(map)
{}

vector<CarState> TrajectoryGenerator::compute_trajectory(CarState& car, double target_speed, Lane target_lane)
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

  // Check: shall egocar stay in the same line or shall it turn to another lane
  if (car.current_lane == target_lane)
  {
    return keep_lane_trajectory(car, anchor_points_x, anchor_points_y, target_speed, ref_car_x, ref_car_y, ref_car_yaw);
  }
  else
  {
    // When the egocar shall change the lane then JMT is used to get a smooth trajectory

    auto s_d_values = _map.get_frenet(ref_car_x, ref_car_y, ref_car_yaw);

    auto s_start  = { s_d_values[0], car.speed_prev/2.24, 0.0 };
    auto s_end    = { s_d_values[0] + 45, target_speed/2.24, 0.0 };

    auto from_d = s_d_values[1];
    auto to_d = car.lane_to_d(target_lane);

    auto d_start = { from_d, 0.0, 0.0 };
    auto d_end = { to_d, 0.0, 0.0 };

    auto t = 1.5; // in seconds

    auto trajectory_in_frenet = switch_lane_trajectory(s_start, s_end, d_start, d_end, 1.5);

    vector<CarState> trajectory;

    auto trajectory_s = trajectory_in_frenet[0];
    auto trajectory_d = trajectory_in_frenet[1];

    assert(trajectory_s.size() == trajectory_d.size());

    for (int i = 0; i < trajectory_s.size(); ++i)
    {
      auto xy_points = _map.get_xy(trajectory_s[i], trajectory_d[i]);

      CarState next_car_state(car.id, xy_points[0], xy_points[1], trajectory_s[0], trajectory_d[i], target_speed*2.24, rad2deg(ref_car_yaw));

      cout << next_car_state.to_string() << endl;

      trajectory.push_back(next_car_state);
    }

    return trajectory;
  }
}

std::vector<CarState> TrajectoryGenerator::keep_lane_trajectory(
  CarState& car,
  std::vector<double> anchor_points_x,
  std::vector<double> anchor_points_y,
  double target_speed,
  double ref_car_x,
  double ref_car_y,
  double ref_car_yaw)
{
  // Now add evenly spaced anchor points in Frenet coordinates
  auto start_s = 30;
  for (auto i = start_s; i <= 90; i += 30)
  {
    auto anchor_point = _map.get_xy(car.s + i, 2 + 4 * static_cast<int>(car.current_lane));
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

  vector<CarState> trajectory;

  // Add all points from the previous time
  for (int i = 0; i < car.prev_waypoints.size(); ++i)
  {
    auto x_point = car.prev_waypoints[i].x;
    auto y_point = car.prev_waypoints[i].y;
    auto s_d_values = _map.get_frenet(x_point, y_point, ref_car_yaw);

    CarState next_car_state(car.id, x_point, y_point, s_d_values[0], s_d_values[1], car.speed, rad2deg(ref_car_yaw));
    trajectory.push_back(next_car_state);
  }

  // starting point in our car's locale coordinate system
  auto x_add_on = 0.0;
  auto a = 3;
  auto current_speed = car.speed_prev;

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

    CarState next_car_state(car.id, x_point, y_point, s_d_values[0], s_d_values[1], current_speed, rad2deg(ref_car_yaw));
    trajectory.push_back(next_car_state);
  }

  car.speed_prev = current_speed;

  assert(trajectory.size() == 50);

  return trajectory;
}

std::vector<std::vector<double>> TrajectoryGenerator::switch_lane_trajectory(
  vector<double> s_start,
  vector<double> s_end,
  vector<double> d_start,
  vector<double> d_end,
  double t)
{
  JMT s_jmt;
  JMT d_jmt;

  s_jmt.set_boundaries(s_start, s_end, t);
  cout << s_jmt.to_string() << endl;

  d_jmt.set_boundaries(d_start, d_end, t);
  cout << d_jmt.to_string() << endl;

  vector<double> s_values;
  vector<double> d_values;

  auto N = int(t / 0.02);

  cout << "JMT N: " << N << endl;

  for (auto i = 0; i < N; ++i)
  {
    auto s = s_jmt(i);
    auto d = d_jmt(i);

    s_values.push_back(s);
    d_values.push_back(d);
  }

  cout << "JMT calculated s and d" << endl;

  return { s_values, d_values };
}
