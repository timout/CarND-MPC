#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "MPC.h"
#include "json.hpp"
#include "helper.h"

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
inline std::string hasData(std::string s) 
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != std::string::npos) {
        return "";
    } else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

inline Eigen::VectorXd vec2eigen(std::vector<double> source) 
{
    return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(source.data(), source.size());
}

int main(int argc, char* argv[]) 
{
    uWS::Hub h;
    const int latency = 100;
    MPC mpc;
    h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        std::string sdata = std::string(data).substr(0, length);
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            std::string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    std::vector<double> ptsx = j[1]["ptsx"];
                    std::vector<double> ptsy = j[1]["ptsy"];
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];
                    
                    double delta = j[1]["steering_angle"];
                    double a = j[1]["throttle"];
                    delta *= -1; //  change of sign because turning left is negative sign in simulator but positive yaw

                    // Convert points to vehicle's coordinate space
                    convert_space(ptsx, ptsy, px, py, psi);

                    // std::vector points vectors to Eigen::VectorXd of points vectors
                    Eigen::VectorXd ptsx_e = vec2eigen(ptsx);
                    Eigen::VectorXd ptsy_e = vec2eigen(ptsy);

                    // Fit a polynomial.
                    Eigen::VectorXd coeffs = polyfit(ptsx_e, ptsy_e, mpc.POLYNOMIAL_ORDER);

                    // Because points were transformed to vehicle coordinates.
                    psi = 0;

                    // Calculates the cross track error
                    // Because points were transformed to vehicle coordinates, x & y equal 0 below.
                    double cte = polyeval(coeffs, 0);
          
                    // Calculate the orientation error
                    // Derivative of the polyfit goes in atan() 
                    // px = 0 in the vehicle coordinates, the higher orders are zero
                    // Leaves only coeffs[1]
                    double epsi = -atan(coeffs[1]);


                    // Computing parameters for latency compensation..
                    double lt = latency / 1000;


                    // Predict state after latency
                    // x, y and psi are all zero after transformation above
                    px = 0.0 + v * lt; // psi is zero, cos(0) = 1
                    py = 0.0;          // sin(0) = 0, y + v * 0 * lt
                    cte = cte + v * sin(epsi) * lt;
                    epsi = epsi + v * delta * lt / mpc.Lf;
                    psi = psi + v * delta * lt / mpc.Lf;
                    v = v + a * lt;

                    Eigen::VectorXd state(6);
                    state << px, py, psi, v, cte, epsi;

                    auto solution = mpc.Solve(state, coeffs);
                    
                    // Prepare JSON message for the simulator environment
                    json msgJson;
                    
                    // New values for the actuators controls
                    msgJson["steering_angle"] = solution->steering;
                    msgJson["throttle"] = solution->accelerator;
                    
                    // points of minimum cost trajectory returned from the solver
                    msgJson["mpc_x"] = solution->x;
                    msgJson["mpc_y"] = solution->y;
                    
                    // Reference points in the vehicle's coordinate system
                    msgJson["next_x"] = solution->next_x;
                    msgJson["next_y"] = solution->next_y;
                    
                    
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
                    std::this_thread::sleep_for(std::chrono::milliseconds(latency));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });
  
    
    h.onConnection([&](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });
    
    h.onDisconnection([&](uWS::WebSocket<uWS::SERVER> ws, int code,
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
