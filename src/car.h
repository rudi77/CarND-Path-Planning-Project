#pragma once
#include <string>
#include <sstream>
#include "helpers.h"
#include <iostream>
#include "waypoint.h"
#include <cassert>

// Represents the state of one car which could either be the ego car itself
// or any other car on the road detected by the sensor modules
class CarState
{
public:
  struct collider 
  {
    bool collision; // is there a collision?
    int  time;      // time collision happens
  };

  // This constructor is used by the ego car
  explicit CarState(int id_) 
    : id(id_), 
      x(0.0), y(0.0),
      s(0.0), d(0.0),
      yaw(0.0), 
      speed(0.0), speed_prev(0.0), 
      current_lane(Lane::Unkown),
      end_path_s(0.0), end_path_d(0.0) {}

  // This constructor is used for the vehicles detected by the car's sensors.
  explicit CarState(int id, double x, double y, double s, double d, double speed, double yaw)
  {
    this->id = id;
    update(x, y, s, d, yaw, speed, 0.0, 0.0);
  }

  void update(double x, double y, double s, double d, double yaw, double speed, double end_path_s, double end_path_d)
  {
    update(x, y, s, d, yaw, speed);
    this->end_path_s = end_path_s;
    this->end_path_d = end_path_d;
  }

  // Updates the state of a vehicle
  void update(double x, double y, double s, double d, double yaw, double speed)
  {
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->yaw = yaw;
    this->speed = speed;
    this->current_lane = d_to_lane(d);
  }

  void set_previous_waypoints(std::vector<double> prev_x, std::vector<double> prev_y)
  {
    assert(prev_x.size() == prev_y.size());

    prev_waypoints.clear();

    for (int i = 0; i < prev_x.size(); ++i)
    {
      prev_waypoints.push_back( {prev_x[i], prev_y[i]} );
    }
  }

  int id;
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  double speed_prev;
  Lane current_lane;
  double end_path_s;
  double end_path_d;
  double accel = 0.0;

  std::vector<WayPoint> prev_waypoints;

  CarState clone(bool with_prev_wps = false) const
  {
    CarState car(id);
    car.x = this->x;
    car.y = this->y;
    car.s = this->s;
    car.d = this->d;
    car.speed = this->speed;
    car.speed_prev = this->speed_prev;
    car.current_lane = this->current_lane;
    car.end_path_s = this->end_path_s;
    car.end_path_d = this->end_path_d;

    if (with_prev_wps)
    {
      if (car.prev_waypoints.size() > 0)
      {
        std::vector<double> prev_x;
        std::vector<double> prev_y;

        for (auto wp : car.prev_waypoints)
        {
          prev_x.push_back(wp.x);
          prev_y.push_back(wp.y);
        }

        car.set_previous_waypoints(prev_x, prev_y);
      }
    }

    return car;
  }

  // checks whether a car is in this car's lane
  bool is_in_lane(const CarState& car) const
  {
    return this->current_lane == car.current_lane;
  }

  // checks whether this car is in front of "car"
  bool is_in_front(const CarState& car) const
  {
    auto check_car_s = static_cast<double>(car.prev_waypoints.size()) * DeltaT * this->speed;
    auto car_s = car.prev_waypoints.size() > 0 ? car.end_path_s : car.s;
    auto in_front =  (this->s + check_car_s) > car_s;

    return in_front;
  }

  bool is_behind(const CarState& car) const
  {
    auto check_car_s = static_cast<double>(car.prev_waypoints.size()) * DeltaT * this->speed;
    auto car_s = car.prev_waypoints.size() > 0 ? car.end_path_s : car.s;

    return (this->s + check_car_s) < car_s;
  }

  // checks whether another car is to too close.
  bool is_too_close(const CarState& car, double safety_buffer = SafetyBufferFront) const
  {
    auto check_car_s = static_cast<double>(car.prev_waypoints.size()) * DeltaT * this->speed;
    auto car_s = car.prev_waypoints.size() > 0 ? car.end_path_s : car.s;
    auto distance = (this->s + check_car_s) - car_s;
    auto too_close = -safety_buffer < distance && distance < safety_buffer;

    return too_close;
  }

  std::tuple<Lane, double, double, double> state_at(double t) {

    /*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
    auto s = this->s + this->speed * t + this->accel * t * t / 2;
    auto v = this->speed + this->accel * t;

    return std::make_tuple(this->current_lane, s, v, this->accel);
  }

  bool collides_with(CarState other, double at_time) const
  {
    auto state_at_t = other.state_at(at_time);
    auto lane       = std::get<0>(state_at_t);
    auto s_at_t     = std::get<1>(state_at_t);

    auto isCollision = (lane == this->current_lane) && abs(s_at_t - this->s) < 2 * Vehicle_Radius;

    if (isCollision)
    {
      std::cout << "OtherCar: " << other.id << " at: " << at_time << " s: " << s_at_t << " lane:" << lane << std::endl;
      std::cout << "Car: "      << this->id << " at: " << at_time << " s: " << this->s << " lane: " << this->current_lane << std::endl;
    }

    return isCollision;
  }


  collider will_collide_with(const CarState& other, int timesteps, double start_time) const
  {
    collider collider_temp;
    collider_temp.collision = false;
    collider_temp.time = -1;

    for (auto t = 0; t < timesteps; ++t)
    {
      if (collides_with(other, start_time))
      {
        collider_temp.collision = true;
        collider_temp.time = t;
        return collider_temp;
      }

      start_time += DeltaT;
    }

    return collider_temp;
  }

  std::string to_string() const
  {
    std::stringstream ss;
    ss << "id: " << id
      << " x: " << x << " y: " << y
      << " s: " << s << " d: " << d
      << " yaw: " << yaw << " v: " << speed
      << " end_s " << end_path_s << " end_d " << end_path_d
      << " lane " << current_lane;
    return ss.str();
  }

  // Creates a car from the sensor data.
  static CarState create(std::vector<double>& sensor_fusion_entry)
  {
    auto id = static_cast<int>(sensor_fusion_entry[0]);
    auto x = sensor_fusion_entry[1];
    auto y = sensor_fusion_entry[2];
    auto dx = sensor_fusion_entry[3];
    auto dy = sensor_fusion_entry[4];
    auto s = sensor_fusion_entry[5];
    auto d = sensor_fusion_entry[6];

    auto yaw = atan2(dy, dx);
    auto speed = sqrt(pow(dx, 2) + pow(dy, 2));

    return CarState(id, x, y, s, d, speed, yaw);
  }

  // Maps a d value to a certain lane.
  static Lane d_to_lane(const double d) 
  {
    if (0.0 <= d && d < 4.0) 
    {
      return Left;
    }
    
    if (4.0 <= d && d < 8.0) 
    {
      return Center;
    }
    
    if (8.0 <= d && d < 12.0) {
      return Right;
    }
    
    return Unkown;
  }

  static double lane_to_d(Lane lane)
  {
    return 2 + 4 * lane;
  }
};

inline void print_trajectories(int car_id, std::vector<CarState> trajectory)
{
  std::string x_vals =
    std::accumulate
    (
      std::next(trajectory.begin()),
      trajectory.end(),
      std::to_string(trajectory[0].x),
      [](std::string a, CarState b) { return a + "," + std::to_string(b.x); }
  );

  std::cout << car_id << "," << x_vals << std::endl;

  std::string y_vals =
    std::accumulate
    (
      std::next(trajectory.begin()),
      trajectory.end(),
      std::to_string(trajectory[0].y),
      [](std::string a, CarState b) { return a + "," + std::to_string(b.y); }
  );

  std::cout << car_id << "," << y_vals << std::endl;
}

inline std::vector<std::vector<double>> to_xy_vectors(const std::vector<CarState> trajectories)
{
  std::vector<double> x_values;
  std::vector<double> y_values;

  for (auto trajectory : trajectories)
  {
    x_values.push_back(trajectory.x);
    y_values.push_back(trajectory.y);
  }

  return { x_values, y_values };
}
