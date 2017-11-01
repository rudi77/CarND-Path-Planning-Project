#include <fstream>
#include <sstream>
#include "map.h"
#include "helpers.h"

using namespace std;

Map::Map(const string& map_file)
{
  // open the file
  ifstream map(map_file.c_str(), ifstream::in);

  // read lines
  string line;

  // One line consists of the global x,y coordinates, the s and d values in Frenet coordinates
  // whereby d is split in an x and y componente.
  while (getline(map, line))
  {
    istringstream iss(line);
    
    double x_, y_, s_, d_x_, d_y_;
    
    iss >> x_;
    iss >> y_;
    iss >> s_;
    iss >> d_x_;
    iss >> d_y_;

    x.push_back(x_);
    y.push_back(y_);
    s.push_back(s_);
    d_x.push_back(d_x_);
    d_y.push_back(d_y_);
  }

  // Set start and endpoints of the route. In our case start and 
  // end point are the same.
  x_start = x[0];
  y_start = y[0];
  x_destination = x[0];
  y_destination = y[0];
}

int Map::closest_waypoint(double x, double y)
{
  auto closestLen = 100000.0; //large number
  auto closestWaypoint = 0;

  for (auto i = 0; i < this->x.size(); i++)
  {
    auto map_x = this->x[i];
    auto map_y = this->y[i];
    auto dist = distance(x, y, map_x, map_y);

    if (dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int Map::next_waypoint(double x, double y, double theta)
{
  auto closestWaypoint = closest_waypoint(x, y);

  auto map_x = this->x[closestWaypoint];
  auto map_y = this->y[closestWaypoint];

  auto heading = atan2((map_y - y), (map_x - x));
  auto angle = abs(theta - heading);

  if (angle > M_PI / 4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> Map::get_frenet(double x, double y, double theta)
{
  auto next_wp = next_waypoint(x, y, theta);
  auto prev_wp = next_wp - 1;

  if (next_wp == 0)
  {
    prev_wp = this->x.size() - 1;
  }

  auto n_x = this->x[next_wp] - this->x[prev_wp];
  auto n_y = this->y[next_wp] - this->y[prev_wp];
  auto x_x = x - this->x[prev_wp];
  auto x_y = y - this->y[prev_wp];

  // find the projection of x onto n
  auto proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
  auto proj_x = proj_norm*n_x;
  auto proj_y = proj_norm*n_y;

  auto frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  auto center_x = 1000 - this->x[prev_wp];
  auto center_y = 2000 - this->y[prev_wp];
  auto centerToPos = distance(center_x, center_y, x_x, x_y);
  auto centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  auto frenet_s = 0.0;

  for (auto i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(this->x[i], this->y[i], this->x[i + 1], this->y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return { frenet_s,frenet_d };

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Map::get_xy(double s, double d)
{
  auto prev_wp = -1;

  while (s > this->s[prev_wp + 1] && (prev_wp < static_cast<int>(this->s.size() - 1)))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % this->x.size();

  auto heading = atan2((this->y[wp2] - this->y[prev_wp]), (this->x[wp2] - this->x[prev_wp]));

  // the x,y,s along the segment
  auto seg_s = (s - this->s[prev_wp]);

  auto seg_x = this->x[prev_wp] + seg_s*cos(heading);
  auto seg_y = this->y[prev_wp] + seg_s*sin(heading);

  auto perp_heading = heading - M_PI / 2;

  auto x = seg_x + d*cos(perp_heading);
  auto y = seg_y + d*sin(perp_heading);

  return { x,y };
}

