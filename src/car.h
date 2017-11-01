#pragma once
#include <string>
#include <sstream>
#include "helpers.h"
#include <iostream>
#include "waypoint.h"
#include <cassert>

// Represents the state of one car which could either be the ego car itself
// or any other car on the road detected by the sensor modules
class Car
{
public:


  // This constructor is used by the ego car
  explicit Car(int id_) 
    : id(id_), 
      x(0.0), y(0.0),
      s(0.0), d(0.0),
      yaw(0.0), 
      speed(0.0), target_speed(0.0),
      current_lane(Lane::Unkown), target_lane(Lane::Unkown) {}

  // This constructor is used for the vehicles detected by the car's sensors.
  explicit Car(int id, double x, double y, double s, double d, double speed, double yaw)
  {
    this->id = id;
    update(x, y, s, d, yaw, speed);
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
    this->target_speed = this->speed;
    this->current_lane = d_to_lane(d);
    this->target_lane = this->current_lane;
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
  double target_speed;

  Lane current_lane;
  Lane target_lane;

  // The simulator updates the state every 20 ms and therefore
  // a timestamp in milliseconds should be sufficient.
  // long timestamp;

  std::vector<WayPoint> prev_waypoints;

  Car clone()
  {
    Car car(id, x, y, s, d, speed, yaw);
    return car;
  }

  std::string to_string() const
  {
    std::stringstream ss;
    ss << "x: " << x << " y: " << y << " s: " << s << " d: " << d << " yaw: " << yaw << " v: " << speed;
    return ss.str();
  }

  // checks whether a car is in this car's lane
  bool is_in_lane(const Car& car) const
  {
    return this->current_lane == car.current_lane;
  }

  // checks whether this car is in front of "car"
  bool is_in_front(const Car& car) const
  {
    return this->s > car.s;
  }

  // checks whether another car is to too close.
  bool is_too_close(const Car& car) const
  {
    auto check_car_s = static_cast<double>(car.prev_waypoints.size()) * 0.02 * this->speed;
    auto distance = this->s - (car.s + check_car_s);
    auto too_close = distance < 30;

    if (too_close)
      std::cout << "Car " << this->id << " is too close, dist " << distance << std::endl;

    return too_close;
  }

  // Creates a car from the sensor data.
  static Car create(std::vector<double>& sensor_fusion_entry)
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

    return Car(id, x, y, s, d, speed, yaw);
  }

  // Maps a d value to a certain lane.
  static Lane d_to_lane(const double d) 
  {
    if (0.0 <= d && d < 4.0) 
    {
      return Lane::Left;
    }
    
    if (4.0 <= d && d < 8.0) 
    {
      return Lane::Center;
    }
    
    if (8.0 <= d && d < 12.0) {
      return Lane::Right;
    }
    
    return Lane::Unkown;
  }
};

inline void print_trajectories(int car_id, std::vector<Car> trajectory)
{
  std::string x_vals =
    std::accumulate
    (
      std::next(trajectory.begin()),
      trajectory.end(),
      std::to_string(trajectory[0].x),
      [](std::string a, Car b) { return a + "," + std::to_string(b.x); }
  );

  std::cout << car_id << "," << x_vals << std::endl;

  std::string y_vals =
    std::accumulate
    (
      std::next(trajectory.begin()),
      trajectory.end(),
      std::to_string(trajectory[0].y),
      [](std::string a, Car b) { return a + "," + std::to_string(b.y); }
  );

  std::cout << car_id << "," << y_vals << std::endl;
}

inline std::vector<std::vector<double>> to_xy_vectors(const std::vector<Car> trajectories)
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
