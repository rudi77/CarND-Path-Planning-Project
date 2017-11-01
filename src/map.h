#pragma once
#include <vector>
#include <string>

///
// The map class contains information about a certain such as waypoints.
// It also provides some helper methods for retrieving waypoints, converting 
// coordinates to and from Frenet coordinates etc.
///
class Map
{
public:
  explicit Map(const std::string& map_file);

  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  std::vector<double> d_x;
  std::vector<double> d_y;

  // Start and endpoints of the route
  double x_start, y_start;
  double x_destination, y_destination;

  // Gets the closest waypoint to (x,y)
  int closest_waypoint(double x, double y);

  int next_waypoint(double x, double y, double theta);

  std::vector<double> get_frenet(double x, double y, double theta);

  std::vector<double> get_xy(double s, double d);
};
