#pragma once

#include <cmath>
#include <stdexcept>

const int EGOCAR = 0xDEADBEEF;

const double SpeedLimit   = 50.0;
const double SpeedBuffer  = 0.5;
const double RefSpeed     = 49.5;
const double DeltaT       = 0.02;
const double DeltaTT      = 0.02*0.02;
const double AccMax       = 10.0;

enum Lane
{
  Left,
  Center,
  Right,
  Unkown
};

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

inline double pos_new(double pos0, double v0, double a)
{
  return pos0 + v0 * DeltaT + 0.5 * a * DeltaTT;
}

inline double v_new(double v0, double a)
{
  return v0 + a * DeltaT;
}

inline Lane get_lane_left(Lane lane)
{
  if (lane == Center)
  {
    return Left;
  }

  if (lane == Right)
  {
    return Center;
  }

  throw std::runtime_error("Invalid lane");
}

inline Lane get_lane_right(Lane lane)
{
  if (lane == Center)
  {
    return Right;
  }

  if (lane == Left)
  {
    return Center;
  }

  throw std::runtime_error("Invalid lane");
}



