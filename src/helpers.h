#pragma once

#include <cmath>
#include <ostream>
#include <functional>
#include <numeric>

namespace Color 
{
  enum Code
  {
    FG_RED = 31,
    FG_GREEN = 32,
    FG_YELLOW = 33,
    FG_BLUE = 34,
    FG_CYAN = 36,
    FG_DEFAULT = 39,
    BG_RED = 41,
    BG_GREEN = 42,
    BG_BLUE = 44,
    BG_DEFAULT = 49
  };

  class Modifier
  {
    Code code;
  public:
    explicit Modifier(Code pCode) : code(pCode) {}

    friend std::ostream&

      operator<<(std::ostream& os, const Modifier& mod)
    {
      return os << "\033[" << mod.code << "m";
    }
  };
}

const Color::Modifier RED(Color::FG_RED);
const Color::Modifier GREEN(Color::FG_GREEN);
const Color::Modifier YELLOW(Color::FG_YELLOW);
const Color::Modifier CYAN(Color::FG_CYAN);
const Color::Modifier DEF(Color::FG_DEFAULT);


const unsigned int EGOCAR = 0xDEADBEEF;

const double MaxS           = 6945.554;
const double SpeedLimit     = 50.0;
const double SpeedBuffer    = 1.0;
const double OptimalSpeed   = 49.5;
const double Horizon        = 1.0;
const double DeltaT         = 0.02;
const double DeltaTT        = 0.02*0.02;
const double AccMax         = 5.0;
const double Ms2Mps         = 2.24;
const double Vehicle_Radius = 2.5;
const double Max_Jerk       = 10.0;   // m / s / s / s

const double SafetyBufferFront = 40.0;
const double SafetyBufferBack = 10.0;

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

inline std::string lane_to_name(Lane lane)
{
  switch (lane)
  {
    case Left:    return "Left";
    case Center:  return "Center";
    case Right:   return "Right";
    case Unkown:
    default:      return "Unkown";
  }
}

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

inline double pos_new(double pos0, double v0, double a, double T)
{
  return pos0 + v0 * T + 0.5 * a *T*T;
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

inline std::function<double(double x)>to_polynom(std::vector<double> coeffs)
{
  return [=](double x)
  {
    auto y = 0.0;
    for (unsigned int i = 0; i < coeffs.size(); ++i)
    {
      y += coeffs[i] * pow(x, i);
    }

    return y;
  };
}

inline std::vector<double> diff_polynome(std::vector<double> coeffs)
{
  std::vector<double> coeffs_new;

  for (unsigned int i = 1; i < coeffs.size(); ++i)
  {
    coeffs_new.push_back(i*coeffs[i]);
  }

  return coeffs_new;
}

template<typename T>
std::string list_to_string( std::vector<T> list)
{
  std::string s = std::accumulate(std::next(list.begin()), list.end(),
    std::to_string(list[0]), // start with first element
    [](std::string a, T b) {
    return a + ", " + std::to_string(b);
  });

  return s;
}
