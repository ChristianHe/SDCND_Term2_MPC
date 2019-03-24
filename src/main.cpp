#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double psi_unity = j[1]["psi_unity"];
#if 0
          // change the reference waypoint from global to vehicle coordinate
          Eigen::VectorXd trans_ptsx(6);
          Eigen::VectorXd trans_ptsy(6);
          for(int i = 0; i < ptsx.size(); i++)
          {
            double diff_x = ptsx[i] - px;
            double diff_y = ptsy[i] - py;
            //theta is down side
            trans_ptsx[i] = (cos(-psi) * diff_x) - (sin(-psi) * diff_y);
            trans_ptsy[i] = (sin(-psi) * diff_x) + (cos(-psi) * diff_y);
          }

          // polyfit the waypointtrans_ptsx[i]
          auto coeffs = polyfit(trans_ptsx, trans_ptsy, 3);

          //calculate the cte and epsi, start at x,y 0, psi 0
          double cte = polyeval(coeffs, 0) - 0.0;
          double epsi = 0.0 - atan(coeffs[1]);

          //prepare the intial state
          VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;
#endif
          //TODO:tackle with the latency
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];
          
          // Need Eigen vectors for polyfit
          Eigen::VectorXd ptsx_car(ptsx.size());
          Eigen::VectorXd ptsy_car(ptsy.size());
          
          // Transform the points to the vehicle's orientation
          for (int i = 0; i < ptsx.size(); i++) {
            double x = ptsx[i] - px;
            double y = ptsy[i] - py;
            ptsx_car[i] = x * cos(-psi) - y * sin(-psi);
            ptsy_car[i] = x * sin(-psi) + y * cos(-psi);
          }
          
          /*
          * Calculate steering angle and throttle using MPC.
          * Both are in between [-1, 1].
          * Simulator has 100ms latency, so will predict state at that point in time.
          * This will help the car react to where it is actually at by the point of actuation.
          */
          
          // Fits a 3rd-order polynomial to the above x and y coordinates
          auto coeffs = polyfit(ptsx_car, ptsy_car, 3);
          
          // Calculates the cross track error
          // Because points were transformed to vehicle coordinates, x & y equal 0 below.
          // 'y' would otherwise be subtracted from the polyeval value
          double cte = polyeval(coeffs, 0);
          
          // Calculate the orientation error
          // Derivative of the polyfit goes in atan() below
          // Because x = 0 in the vehicle coordinates, the higher orders are zero
          // Leaves only coeffs[1]
          double epsi = -atan(coeffs[1]);
          
          // Center of gravity needed related to psi and epsi
          const double Lf = 2.67;
          
          // Latency for predicting time at actuation
          const double dt = 0.1;
          
          // Predict state after latency
          // x, y and psi are all zero after transformation above
          double pred_px = 0.0 + v * dt; // Since psi is zero, cos(0) = 1, can leave out
          const double pred_py = 0.0; // Since sin(0) = 0, y stays as 0 (y + v * 0 * dt)
          double pred_psi = 0.0 + v * -delta / Lf * dt;
          double pred_v = v + a * dt;
          double pred_cte = cte + v * sin(epsi) * dt;
          double pred_epsi = epsi + v * -delta / Lf * dt;
          
          // Feed in the predicted state values
          Eigen::VectorXd state(6);
          state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;

          auto vars = mpc.Solve(state, coeffs);

          // elements in vars: delta, a, x1, y1, x2, y2, ..., xN, yN

          /**
           * Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */
          double steer_value;
          double throttle_value;

          steer_value = vars[0] / deg2rad(25);
          throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the 
          //   steering value back. Otherwise the values will be in between 
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          /**
           *   add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Green line
           */
          for(int i = 2; i < vars.size(); i+=2)
          {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i+1]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           *   add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Yellow line
           */
          //cout << "ptsx size: " << ptsx.size() << endl;
          for(int i = 0; i < 100; i = i+10)
          {
            next_x_vals.push_back(i);
            next_y_vals.push_back(polyeval(coeffs, i));
          }
          
          // transform the reference point (next_x_vals, next_y_vals) from unity global 
          // coordinate to vehicle coordinate.
          // psi_unity shall be used, 
          // TODO:check the theta meaning.

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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