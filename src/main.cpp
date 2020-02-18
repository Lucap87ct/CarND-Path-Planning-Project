#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <fstream>
#include <iostream>
#include <string>
#include <uWS/uWS.h>
#include <vector>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  constexpr double max_s = 6945.554;

  bool initialization{
      true}; // boolean to determine if we are in the "cold start" phase
  double ref_vel{0.0}; // ego vehicle reference velocity

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &initialization,
               &ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data,
                         size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message
    // event. The 4 signifies a websocket message The 2 signifies a
    // websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // declaring some constants
          constexpr double vel_increment{0.224}; // velocity increment in mph
          constexpr double mph_to_mps = 1.61 / 3.6;
          constexpr double points_s_spacing{0.02}; // m
          constexpr int min_prev_size = {
              2}; // min size of previous points for initialization
          constexpr double waypoints_s_spacing{30.0}; // m
          constexpr double safety_distance_front{
              30.0}; // safety distance in s from front objects in m
          constexpr double safety_distance_overtake{
              30.0}; // safety distance in s from side objects for overtaking in
          // m
          constexpr double safety_distance_overtake_stationary{
              10.0}; // safety distance in s from side objects for overtaking in
                     // m
          constexpr int n_points{50};       // number of points on the spline
          constexpr double vel_limit{49.5}; // hard-coded speed limit

          std::size_t prev_size =
              previous_path_x
                  .size(); // how many points in the previous path vector

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          const int left_lane{0};
          const int center_lane{1};
          const int right_lane{2};
          // current lane calculation
          int current_host_lane{center_lane};
          if (car_d < (2 + 4 * center_lane + 2) &&
              car_d > (2 + 4 * center_lane - 2)) {
            current_host_lane = center_lane;
          } else if (car_d < (2 + 4 * left_lane + 2) &&
                     car_d > (2 + 4 * left_lane - 2)) {
            current_host_lane = left_lane;
          }
          if (car_d < (2 + 4 * right_lane + 2) &&
              car_d > (2 + 4 * right_lane - 2)) {
            current_host_lane = right_lane;
          }

          int ref_lane = current_host_lane; // reference lane is the center lane

          // basic overtaking rules
          bool overtaking_lane_left_free{false};
          bool overtaking_lane_right_free{false};
          if (current_host_lane == center_lane) {
            overtaking_lane_left_free = true;
            overtaking_lane_right_free = true;
          } else if (current_host_lane == left_lane) {
            overtaking_lane_right_free = true;
          } else if (current_host_lane == right_lane) {
            overtaking_lane_left_free = true;
          }

          // reference velocity calculation from object fusion data
          // together with reference lane calculation for overtaking
          bool close_object{false};
          double v_min_close_object{std::numeric_limits<double>::max()};
          double vx_object, vy_object, vabs_object, s_object, d_object;
          double s_object_without_prediction;
          for (std::size_t i = 0; i < sensor_fusion.size(); i++) {
            d_object = sensor_fusion[i][6];
            vx_object = sensor_fusion[i][3];
            vy_object = sensor_fusion[i][4];
            vabs_object =
                std::sqrt(vx_object * vx_object + vy_object * vy_object);
            s_object = sensor_fusion[i][5];
            s_object_without_prediction = s_object;
            // simple object prediction
            s_object += (double)prev_size * points_s_spacing * vabs_object;
            // check if object is in ego lane and extract its velocity and s
            if (d_object < (4 + 4 * current_host_lane) &&
                d_object > (4 * current_host_lane)) {
              if ((s_object > car_s) &&
                  (s_object - car_s < safety_distance_front)) {
                close_object = true;
                std::cout << "Close object in front" << std::endl;
                if (vabs_object < v_min_close_object) {
                  v_min_close_object = vabs_object;
                }
              }
            }
            // check if object is left of host vehicle within a safety s
            // distance
            else if ((d_object < 4 * current_host_lane) &&
                     (d_object > 4 * current_host_lane - 2)) {
              if (abs(s_object - car_s) < safety_distance_overtake ||
                  abs(s_object_without_prediction - car_s) <
                      safety_distance_overtake_stationary) {
                overtaking_lane_left_free = false;
                std::cout << "Left lane is occupied" << std::endl;
              }
            }
            // check if object is right of host vehicle within a safety s
            // distance
            else if ((d_object > 4 + 4 * current_host_lane) &&
                     (d_object < 6 + 4 * current_host_lane)) {
              if (abs(s_object - car_s) < safety_distance_overtake ||
                  abs(s_object_without_prediction - car_s) <
                      safety_distance_overtake_stationary) {
                overtaking_lane_right_free = false;
                std::cout << "Right lane is occupied" << std::endl;
              }
            } else {
              std::cout << "Left and right lanes are free" << std::endl;
            }
          }

          // reference velocity final calculation
          if (initialization) {
            ref_vel = 0.0;
            initialization = false;
          }

          if (close_object) {
            if (car_speed > v_min_close_object) {
              ref_vel -= vel_increment;
            }
            // prefer left overtaking to right overtaking
            if (overtaking_lane_left_free && current_host_lane != left_lane) {
              ref_lane = current_host_lane - 1;
            } else if (overtaking_lane_right_free &&
                       current_host_lane != right_lane) {
              ref_lane = current_host_lane + 1;
            }
          } else if (ref_vel < vel_limit) {
            ref_vel += vel_increment;
          }

          // points for generating the spline
          vector<double> ptsx;
          vector<double> ptsy;

          // current car state
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // first two points of the spline: when previous path vector is
          // not initialized, they are the current car position and its
          // back-propagation with current heading
          if (prev_size < min_prev_size) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }

          // first two points of the spline: when previous path vector is
          // initialized, they are the last two points from the
          // reference path and calculate the reference angle with
          // back-propagation
          else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            double ref_x_prev = previous_path_x[prev_size - min_prev_size];
            double ref_y_prev = previous_path_y[prev_size - min_prev_size];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // next three points of the spline: calculate them from the
          // current car s coordinate and d equal to center of ref_lane
          vector<double> next_wp0 =
              getXY(car_s + waypoints_s_spacing, 2 + 4 * ref_lane,
                    map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 =
              getXY(car_s + 2 * waypoints_s_spacing, 2 + 4 * ref_lane,
                    map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 =
              getXY(car_s + 3 * waypoints_s_spacing, 2 + 4 * ref_lane,
                    map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // calculate the spline points in car local reference frame
          for (std::size_t i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          // set the spline to interpolate among the calculated points
          tk::spline s{};
          s.set_points(ptsx, ptsy);

          // here it will be stored the next values for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // next values for the planner: the first values are from the
          // previous path
          for (std::size_t i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // determine N points from the spline
          double target_x = waypoints_s_spacing;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;

          for (std::size_t i = 1; i <= n_points - previous_path_x.size(); i++) {
            double N = target_dist / (points_s_spacing * ref_vel * mph_to_mps);
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;
            double x_ref = x_point;
            double y_ref = y_point;

            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            // next values for the planner: the other points are calculated
            // from the spline
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
  }); // end h.onMessage

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
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
