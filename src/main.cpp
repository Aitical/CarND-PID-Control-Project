#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid_steering;
  PID pid_throttle;

  /**
   * Initialize the pid variable.
   */
  double kp_steer = -.1;
  double ki_steer = -.00015;
  double kd_steer = -1.0;

  double kp_throt = .15;
  double ki_throt = 0.;
  double kd_throt = 0.6;

  pid_steering.Init(kp_steer, ki_steer, kd_steer);
  pid_throttle.Init(kp_throt, ki_throt, kd_throt);


  h.onMessage([&pid_steering,&pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
		      double throt_value;
		      // steer set limit
		      double max_steer_value = .5;
		      double max_throt_value = .6;

		      // update errors
		      pid_steering.UpdateError(cte);
		      pid_throttle.UpdateError(fabs(cte));

          // Calculate values
		      steer_value = pid_steering.TotalError();
		      throt_value = pid_throttle.Kp * fabs(pid_throttle.p_error) + pid_throttle.Kd * fabs(pid_throttle.d_error);

		      // check values if they overflow
		      if (fabs(steer_value) > max_steer_value) {
			        steer_value = fabs(steer_value) / steer_value * max_steer_value;
		      }
		      if (fabs(throt_value) > max_throt_value) {
			      throt_value = fabs(throt_value) / throt_value * max_throt_value;
		      }
		      // recovery in case of low speed motion
		      if (fabs(speed) < 12) {
			      throt_value = -6.;
		      }
			  

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.4 - throt_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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