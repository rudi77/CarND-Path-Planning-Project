#pragma once

#include <cmath>

inline double deg2rad(double x) { return x * M_PI / 180; }

inline double rad2deg(double x) { return x * 180 / M_PI; }

inline double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

inline double distance(double x, double y)
{
  return sqrt(x*x + y*y);
}

enum Lane
{
  Left,
  Center,
  Right
};
