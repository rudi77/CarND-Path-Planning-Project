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
#include "road.h"
#include "car.h"
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
 
  Map map("../data/highway_map.csv");
  CarState car(EGOCAR);
  Road road(car);
  BehaviorPlanner behavior_planner(car);

  h.onMessage([&map, &road, &car, &behavior_planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double x = j[1]["x"];
          double y = j[1]["y"];
          double s_ = j[1]["s"];
          double d = j[1]["d"];
          double yaw = j[1]["yaw"];
          double speed = j[1]["speed"];

          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          car.update(x, y, s_, d, yaw, speed, end_path_s, end_path_d);

          auto prev_path_x = j[1]["previous_path_x"];
          auto prev_path_y = j[1]["previous_path_y"];

          // Previous path data given to the Planner
          car.set_previous_waypoints(prev_path_x, prev_path_y);

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          vector<CarState> other_cars;

          for (vector<double> entry : sensor_fusion)
          {
            other_cars.push_back(CarState::create(entry));
          }

          road.update(other_cars);

          auto trajectory = behavior_planner.transition(other_cars, map, road);

          json msgJson;
          msgJson["next_x"] = trajectory[0];
          msgJson["next_y"] = trajectory[1];

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          // simulate a certain delay
          //this_thread::sleep_for(chrono::milliseconds(100));

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
