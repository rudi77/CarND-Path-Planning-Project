#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "helpers.h"
#include "map.h"
#include "car.h"
#include "trajectory_generator.h"
#include "predictor.h"
#include "behavior_planner.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  auto max_s = 6945.554;

  Map map(map_file_);
  Car car(EGOCAR);
  car.target_speed = RefSpeed;

  BehaviorPlanner behavior_planner(car, max_s);

  h.onMessage([&map, &car, &behavior_planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          car.update(j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], j[1]["speed"]);

          auto prev_path_x = j[1]["previous_path_x"];
          auto prev_path_y = j[1]["previous_path_y"];

          // Previous path data given to the Planner
          car.set_previous_waypoints(prev_path_x, prev_path_y);

          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          vector<Car> other_cars;

          for (vector<double> entry : sensor_fusion)
          {
            other_cars.push_back(Car::create(entry));
          }

          // START PREDICTIONS

          // Calc trajectories for each vehicle
          auto trajectory_predictions = Predictor::predict_trajectories(other_cars, map);

          // END PREDICTIONS

          // START Behavoir planning -------------
          // Check for possible collisions - car is too close in the future
          // Therefore iterate over all other vehciles and check if there
          // might be a possible collision.
          //auto too_close = false;

          //for (auto other_car : other_cars)
          //{
          //  if (other_car.is_in_lane(car))
          //  {
          //    // if other_car is in front of ego car and distance is lower than x meters
          //    // then either slow down or try to change the lane.
          //    if (other_car.is_in_front(car) && other_car.is_too_close(car))
          //    {
          //      too_close = true;
          //      //car.target_speed = other_car.speed;
          //    }
          //  }
          //}

          //if (too_close)
          //{
          //  //car.speed -= 0.225;
          //  //car.target_speed = RefSpeed;
          //  car.target_lane = get_lane_left(car.current_lane);

          //  cout << "car currentlane: " << car.current_lane << " targetlane: " << car.target_lane << endl;

          //}
          //else if (car.speed < RefSpeed)
          //{
          //  //car.speed += 0.225;
          //  car.target_speed = RefSpeed;
          //}
          
          //TrajectoryGenerator generator(map);
          //auto trajectory = generator.compute_trajectory(car);

          auto trajectory = behavior_planner.transition(trajectory_predictions, map);
          // END BEHAVIOR PLANNING -------------

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (auto t : trajectory)
          {
            //next_x_vals.push_back(t.x);
            //next_y_vals.push_back(t.y);
            next_x_vals.push_back(t[0]);
            next_y_vals.push_back(t[1]);
          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          // simulate a certain delay
          this_thread::sleep_for(chrono::milliseconds(100));

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      }
      else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
    size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    }
    else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
    char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  }
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
