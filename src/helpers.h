#pragma once

#include <cmath>

const int EGOCAR = 0xDEADBEEF;

const double MaxS           = 6945.554;
const double SpeedLimit     = 50.0;
const double SpeedBuffer    = 0.5;
const double OptimalSpeed   = 49.5;
const double DeltaT         = 0.02;
const double DeltaTT        = 0.02*0.02;
const double AccMax         = 10.0;
const double Ms2Mps         = 2.24;
const double Vehicle_Radius = 1.5;
const double Max_Jerk       = 10.0;   // m / s / s / s

const double Expected_Jerk_In_One_Sec = 2.0; // m / s / s
const double Expected_Acc_In_One_Sec = 1.0;  // m / s

const std::vector<double> SIGMA_S = { 10.0, 3.0, 0.1 };
const std::vector<double> SIGMA_D = { 0.0, 0.0, 0.0 };

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

inline double round2(double x, int decimals)
{
  auto d = pow(10.0, decimals);
  return round(x * d) / d;
}

inline std::vector<double> quadratic_solver(double c, double b, double a)
{
  // Check: a must not be 0

  auto x1 = (-1 * b + sqrt(b*b - 4 * a*c)) / 2 * a;
  auto x2 = (-1 * b - sqrt(b*b - 4 * a*c)) / 2 * a;

  return { x1, x2 };
}
