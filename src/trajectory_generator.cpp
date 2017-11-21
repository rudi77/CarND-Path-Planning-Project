#include <cassert>
#include <iostream>

#include "trajectory_generator.h"
#include "car.h"
#include "helpers.h"
#include "spline.h"
#include "jmt.h"

using namespace std;

TrajectoryGenerator::TrajectoryGenerator(const Map& map) : _map(map)
{}

vector<CarState> TrajectoryGenerator::compute_trajectory(const CarState& car, double target_speed, Lane target_lane)
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

  auto current_speed = car.speed_prev;
  auto N = Horizon / DeltaT;

  add_anchor_points_change_lane(car, target_speed, current_speed, N, target_lane, anchor_points_x, anchor_points_y);

  // Shifting anchor points into car reference points
  for (unsigned int i = 0; i < anchor_points_x.size(); ++i) {
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
  for (unsigned int i = 0; i < car.prev_waypoints.size(); ++i)
  {
    auto x_point = car.prev_waypoints[i].x;
    auto y_point = car.prev_waypoints[i].y;
    auto s_d_values = _map.get_frenet(x_point, y_point, ref_car_yaw);

    CarState next_car_state(car.id, x_point, y_point, s_d_values[0], s_d_values[1], car.speed, rad2deg(ref_car_yaw));
    trajectory.push_back(next_car_state);
  }

  // starting point in our car's locale coordinate system
  auto x_add_on = 0.0;
  auto maxAcc = AccMax;

  for (auto i = 1; i <= N - car.prev_waypoints.size(); ++i)
  {
    auto a = maxAcc;
    
    // reduce speed if it is above target_speed
    if (current_speed > target_speed)
    {
      a = -1 * a;
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

  return trajectory;
}

void TrajectoryGenerator::add_anchor_points_change_lane(
  const CarState& car,
  double& target_speed,
  double current_speed,
  double& N,
  Lane target_lane,
  vector<double>& anchor_points_x,
  vector<double>& anchor_points_y)
{
  auto T = Horizon + 1.0;
  auto a = AccMax;
  auto a_end = AccMax;

  // Use JMT to generate possible trajectories
  auto s = car.prev_waypoints.size() == 0 ? car.s : car.end_path_s;
  auto d = car.prev_waypoints.size() == 0 ? car.d : car.end_path_d;

  if (target_speed >= OptimalSpeed)
  {
    target_speed = OptimalSpeed;
    a = 0.0;
    a_end = 0.0;
  }
  else
  {
    // check if we can reach the target_speed within
    // the given T
    if (current_speed < target_speed)
    {
      auto v = current_speed / Ms2Mps + a * T;
      if (v < target_speed)
      {
        target_speed = v;
      }
      else
      {
        if (v >= OptimalSpeed)
        {
          target_speed = OptimalSpeed;
          a_end = 0.0;
        }
      }
    }
  }

  // how far can we go in T seconds?
  auto s_end_pos = pos_new(s, target_speed / Ms2Mps, 0.0, T);
  auto d_end_pos = 2 + 4 * static_cast<double>(target_lane);

  vector<double> start_s  = { s, current_speed / Ms2Mps, a };
  vector<double> end_s    = { s_end_pos, target_speed / Ms2Mps, a_end };
  vector<double> start_d  = { d, 0.0, 0.0 };
  vector<double> end_d    = { d_end_pos, 0.0, 0.0 };

  auto coeff_s = JMT()(start_s, end_s, T);
  auto coeff_d = JMT()(start_d, end_d, T);

  auto poly_s = to_polynom(coeff_s);
  auto poly_d = to_polynom(coeff_d);

  auto next_s = poly_s(T);
  auto next_d = poly_d(T);
  auto anchor_point = _map.get_xy(next_s, next_d);

  anchor_points_x.push_back(anchor_point[0]);
  anchor_points_y.push_back(anchor_point[1]);

  // add another anchor point
  auto anchor_point2 = _map.get_xy(s_end_pos + 30, 2 + 4 * static_cast<int>(target_lane));
  anchor_points_x.push_back(anchor_point2[0]);
  anchor_points_y.push_back(anchor_point2[1]);
}

vector<double> TrajectoryGenerator::perturb_data(vector<double> means, vector<double> sigmas)
{
  assert(means.size() == sigmas.size());

  random_device rd;
  mt19937 e2(rd());

  vector<double> new_means;

  for (unsigned int i = 0; i < sigmas.size(); ++i)
  {
    normal_distribution<> gauss(means[i], sigmas[i]);
    new_means.push_back(gauss(e2));
  }

  return new_means;
}

