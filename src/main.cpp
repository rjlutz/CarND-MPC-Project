#include <math.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <uWS/uWS.h>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }

bool debug_flag = false;

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

double alpha_v = -1.0; // use -1 to flag uninitialized lowpass filter
double alpha_a = -1.0;
double prevailing_v;   // remember historical for lowpass
double prevailing_a;

double lowpass(double alpha, double current, double prevailing) {
  double filtered = 0.0;
  if (alpha < 1) { // establish history as first value
    alpha = 0.9;
    filtered = current;
  } else {
    filtered = current * (1-alpha) + prevailing * alpha;
  }
  return filtered;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc(debug_flag);

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    if (debug_flag) cout << sdata << endl;
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
          double delta = j[1]["steering_angle"]; //est from sim, could be used for handling latency
          double a = j[1]["throttle"];

          for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - px; // shift the car reference angle to 90 degrees
            double shift_y = ptsy[i] - py;
            ptsx[i] = shift_x * cos(0-psi) - shift_y * sin(0-psi);
            ptsy[i] = shift_x * sin(0-psi) + shift_y * cos(0-psi);
          }

          Eigen::Map<Eigen::VectorXd> ptsx_transform(&ptsx[0], 6);
          Eigen::Map<Eigen::VectorXd> ptsy_transform(&ptsy[0], 6);

          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

          // calculate cte and epsi
          double cte = polyeval(coeffs,0); // 0 b/c we shifted car to 0,0,0; is an approximation
          // potential improvement, calculate the shortest distance to poly'l line exactly

          // double epsi = psi - atan(coeffs[1] + 2 * px * coeffs[2] +
          //               3 * coeffs[3] * pow(px,2));
          // simplifies to:
          double epsi = -atan(coeffs[1]);

          double dt_delay = 0.1;
          double Lf = 2.67;

          prevailing_v = lowpass(alpha_v, v, prevailing_v);
          prevailing_a = lowpass(alpha_a, a, prevailing_a);

          Eigen::VectorXd state(6);
          // predict for state 'dt_delay' time units, in the future
          state << prevailing_v * dt_delay,
                  0,
                  prevailing_v * -delta/Lf *dt_delay,
                  prevailing_v + prevailing_a * dt_delay,
                  cte + prevailing_v * sin(epsi) * dt_delay,
                  epsi + prevailing_v + prevailing_a * -delta/Lf * dt_delay;

          auto vars = mpc.Solve(state, coeffs);

          // Green (MPC predicted) trajectory, relative to the vehicle's
          // coordinate system
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          for (int i = 2; i < vars.size(); i++) {
            if (i%2 == 0)
              mpc_x_vals.push_back(vars[i]);
            else
              mpc_y_vals.push_back(vars[i]);
          }

          // Yellow  reference line, pts relative to the vehicle's
          // coordinate system. for starts at 2, no need to draw under car
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          double poly_inc = 2.5;
          int num_points = 25;
          for (int i = 2; i < num_points; i++) {
            next_x_vals.push_back(poly_inc*i);
            next_y_vals.push_back(polyeval(coeffs, poly_inc*i));
          }

          json msgJson;
          msgJson["steering_angle"] = vars[0]/(deg2rad(25)*Lf); // xform angle to [-1, 1]
          msgJson["throttle"] = vars[1]; // steering angle and throttle from MPC
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          if (debug_flag) cout << msg << endl;
          if (debug_flag) cout << "steering, throttle =" <<  vars[0]/(deg2rad(25)*Lf)<< " " << vars[1] << endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
