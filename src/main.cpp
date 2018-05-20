#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;
using namespace Eigen;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  /**
   * Model-predictive controller instantiated
   */
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
		  /**
		   * Reading the data from the simulator's JSON object:
		   * ptsx - global x position of the waypoints
		   * ptsy - global y position of the waypoints
		   * psi - car's orientation
		   * speed - car's velocity
		   * stering_angle - current steering angle in radians
		   * throttle - current throttle in range [-1, 1]
		   */
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          /**
           * transforming waypoints' coordinates from
           * global coordinates to car coordinates
           */
          VectorXd ptsx_local(ptsx.size());
          VectorXd ptsy_local(ptsy.size());
          for (unsigned i = 0; i < ptsx.size(); i++) {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            ptsx_local[i] = dx * cos(-psi) - dy * sin(-psi);
            ptsy_local[i] = dx * sin(-psi) + dy * cos(-psi);
          }

          /**
           * fitting 3rd degree polynomial to the waypoints
           */
          VectorXd coeffs = polyfit(ptsx_local, ptsy_local, 3);

          /**
           * compute cross track error
           */
          double cte = polyeval(coeffs, 0);

          /**
           * compute orientation error
           */
          double epsi = -atan(coeffs[1]);

          /**
           * state vector
           */
          VectorXd state(num_states);

          /**
           * State vector taking into account latency of 0.1s=100ms
           */
          state << 0 + v * latency,
        		   0 ,
				   0 - v * delta / Lf * latency,
				   v + a * latency,
				   cte + v * sin(epsi) * latency,
				   epsi + v * - delta / Lf * latency;

          /**
           * computing steering angle and throttle values
           */
          vector<double> mpc_out = mpc.Solve(state, coeffs);


          /**
           * forming json message to send back to the simulator,
           * with new steering and throttle controls
           */
          json msgJson;
          double steer_value = mpc_out[0];
          double throttle_value = mpc_out[1];
          msgJson["steering_angle"] = steer_value/( deg2rad(25) * Lf );
          msgJson["throttle"] = throttle_value;

          /**
           * JSON message:
           * 'mpc_x' and 'mpc_y' -- green line displayed in the simulator
           * State vector 'x' and 'y' from the solution of the MPC
           */
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          for (unsigned i=2; i < 2+N; ++i){
        	  mpc_x_vals.push_back(mpc_out[i]);
          }
          for (unsigned i=2+N; i < 2+2*N; ++i){
        	  mpc_y_vals.push_back(mpc_out[i]);
          }
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          /**
           * JSON message:
           * 'next_x' and 'next_y' -- yellow line displayed in the simulator
           * Waypoints of the previously fitted polynomial
           */
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for (unsigned i = 0; i < 50; i++){
            next_x_vals.push_back(i);
            next_y_vals.push_back(polyeval(coeffs, (double) i));
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          /**
           * Apply latency
           */
          this_thread::sleep_for(chrono::milliseconds(100));
          /**
           * Send the JSON message to the simulator
           */
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
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
    } else {
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
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
