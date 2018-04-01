#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "MPC.h"
#include "json.hpp"
#include "polyutils.h"
#include <cassert>
// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Convert from map to vehcile coordinate for X
inline double map_to_vehicle_coordinate_x(double mx, double my,
                                          double vx, double vy, double vpsi) {
  return (my - vy) * sin(vpsi) + (mx - vx) * cos(vpsi);
}

// Convert from map to vehcile coordinate for Y
inline double map_to_vehicle_coordinate_y(double mx, double my,
                                          double vx, double vy, double vpsi) {
  return (my - vy) * cos(vpsi) - (mx - vx) * sin(vpsi);
}

int main() {
  uWS::Hub h;

  size_t counter = 0;
  double total_cost = 0;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc, &counter, &total_cost](
    uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (! (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')) {
      return;
    }

    string s = hasData(sdata);
    if (s == "") {
      // Manual driving
      std::string msg = "42[\"manual\",{}]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      return;
    }

    auto j = json::parse(s);
    string event = j[0].get<string>();
    if (event != "telemetry") {
      return;
    }

    // j[1] is the data JSON object
    vector<double> ptsx = j[1]["ptsx"];
    vector<double> ptsy = j[1]["ptsy"];
    double px = j[1]["x"];
    double py = j[1]["y"];
    double psi = j[1]["psi"];
    double v = j[1]["speed"];
    double delta = j[1]["steering_angle"];
    double a = j[1]["throttle"];

    // Steering angle sent back from Unity is of the opposite direction:
    //   i.e. positive steering angle signifies right turn where as in our vehicle
    //   model it means left turn.
    delta *= -1;

    // Translate waypoints from map to vehicle coordinate system.
    for (int i = 0; i < ptsx.size(); i++) {
      double mx = ptsx[i],
             my = ptsy[i];
      ptsx[i] = map_to_vehicle_coordinate_x(mx, my, px, py, psi);
      ptsy[i] = map_to_vehicle_coordinate_y(mx, my, px, py, psi);
    }
    // Now the coordinate origin is the car itself.
    px = py = psi = 0;

    // Fit a third-degree polynomial to the translated waypoints.
    auto coeffs = poly_fit(ptsx, ptsy, 3);

    // Deal with actuator latency by predicting the vehicle states in the future
    // using the kinematic model and initialize MPC with the future states.
    const long latencyMs = 100;
    const double latency = latencyMs / 1000.0;
    // Here we assume latency is very small.
    px += v * latency;
    py += v * latency * delta * latency;
    psi += v * delta / Lf * latency;
    // For error terms, use (actual - reference)
    double cte = py - poly_eval(coeffs, px);
    double epsi = psi - atan(poly_deriv_1(coeffs, px));
    // Update velocity last.
    v += a * latency;

    // Assumble the initiate states.
    Eigen::VectorXd state(6);
    state << px,   // Vehicle's x coorindate
             py,   // Vehicle's y coorindate
             psi,  // Vehicle's heading direction
             v,    // Vehicle velocity
             cte,  // Crosstrek error
             epsi; // Heading error

    // Solve MPC problem.
    auto res = mpc.Solve(state, coeffs, deg2rad(25), Lf);

    counter += 1;
    total_cost += res.cost;
    printf("Running average of cost: %f\n", total_cost / counter);

    json msgJson;

    // Scale the steering angle to [-1, 1]. Negate the sign with conform with Unity.
    msgJson["steering_angle"] = -res.steering / (deg2rad(25) * Lf);
    msgJson["throttle"] = res.throttle;

    /*
     * Display MPC predicted trajectory, whose length is proportional to the velocity.
     * In the simulation, these points will be connected by a green line.
     */
    msgJson["mpc_x"] = res.xpos;
    msgJson["mpc_y"] = res.ypos;

    /*
     * Display the waypoints/reference line. Use the fitted polynomial.
     * In the simulation, these points will be connected by a yellow line.
     */
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    const double x_inc = 2.5; /* meters. */
    const int num_points = 40;
    for (int i = 0; i < num_points; i++) {
      next_x_vals.push_back(x_inc * i);
      next_y_vals.push_back(poly_eval(coeffs, x_inc * i));
    }
    msgJson["next_x"] = next_x_vals;
    msgJson["next_y"] = next_y_vals;


    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
    // std::cout << msg << std::endl;
    // Latency
    // The purpose is to mimic real driving conditions where
    // the car does actuate the commands instantly.
    //
    // Feel free to play around with this value but should be to drive
    // around the track with 100ms latency.
    //
    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
    // SUBMITTING.
    this_thread::sleep_for(chrono::milliseconds(latencyMs));
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
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
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
