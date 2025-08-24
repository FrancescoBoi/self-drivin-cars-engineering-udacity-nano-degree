#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "twiddle.h"
//#include "twiddle.h"

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
  Twiddle tw;
  //PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
   /*
   Kp: 0.00300216
Kd: 0.44998
Ki: 1.00207e-05
dKp: 3.58999e-08
dKd: 1.96627e-06
dKi: 3.19869e-10
*/
/*running for throttle 0.7
Kp: 0.00418979
Kd: 0.463819
Ki: 2.75867e-05
dKp: 1.65819e-07
dKd: 2.47703e-06
dKi: 2.78006e-09*/
//throttle 0.5:  0.02, 0.8, 2.75867e-05
/*throttle 0.5 Kp: 0.0406096414366716
Kd: 1.8481780107309
Ki: 0.000159605301845654
dKp: 1.95827944777816e-06
dKd: 0.000118195193007186
dKi: 1.34312719326348e-08*/
/*throttle 0.6:
Kp: 0.0406076831572238
Kd: 1.8481780107309
Ki: 0.000159618733117587
dKp: 3.13547206895138e-09
dKd: 1.72042363179773e-07
dKi: 1.75952416888405e-11
*/

/* throttle 0.7
Kp: 0.0406077122567417
Kd: 1.84818009331409
Ki: 0.000159618906729345
dKp: 8.50195221303102e-12
dKd: 9.46366024579648e-10
dKi: 5.94964079631233e-14
*/

   tw.Init(0.0406076831572238, 1.8481780107309, 0.000159618733117587,
       8.50195221303102e-12, 9.46366024579648e-10, 5.94964079631233e-14);

   tw.state = tuned;
   //tw.Init(0.00300216, 0.44998, 1.00207e-05, 0.0001, 0.01, 0.000001);
   tw.printK();

  h.onMessage([&tw](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode)
                     {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    tw.setWS(ws);
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

          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          // DEBUG

          if (abs(speed)<0.1) //if the vehicle get stucked set max error and reset
          {
              tw.setMaxError();
              tw.reset();
          }
          tw.UpdateError(pow(cte,3));
          double steer_value = tw.TotalError();
          if (tw.isTuned())
          {
              std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                    << std::endl;
          }
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.6;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          if (tw.isTuned())
          {
              std::cout << msg << std::endl;
          }
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "=======\n\nConnected!!!" << std::endl;
  });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
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
