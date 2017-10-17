#pragma once

#include <string>
#include <vector>
#include <cmath>

///
// The map class contains information about a certain such as waypoints.
// It also provides some helper methods for retrieving waypoints, converting 
// coordinates to and from Frenet coordinates etc.
///
class Map
{
public:
  Map(const std::string& map_file);

  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  std::vector<double> d_x;
  std::vector<double> d_y;

  // Gets the closest waypoint to (x,y)
  int closest_waypoint(double x, double y);

  int next_waypoint(double x, double y, double theta);

  std::vector<double> get_frenet(double x, double y, double theta);

  std::vector<double> get_xy(double s, double d);

  static double distance(double x1, double y1, double x2, double y2)
  {
    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
  }
};
