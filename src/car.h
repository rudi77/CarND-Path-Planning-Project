#pragma once
#include "helpers.h"
#include <string>
#include <sstream>

class Car
{
public:
  int id;

  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;

  Lane current_lane;

  std::string to_string() const
  {
    std::stringstream ss;
    ss << "x: " << x << " y: " << y << " s: " << s << " d: " << d << " yaw: " << yaw << " v: " << speed;
    return ss.str();
  }
};
