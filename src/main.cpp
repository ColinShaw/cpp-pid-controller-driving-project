#include <iostream>
#include <math.h>
#include <uWS/uWS.h>
#include "json.hpp"
#include "PID.h"
#include "Tune.h"


#define TUNE_CONTROLLER   false
#define FIRST_SAMPLE_STEP 200
#define LAST_SAMPLE_STEP  500


using json = nlohmann::json;

std::string hasData(std::string s) 
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) 
  {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) 
  {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  int step = 0;
  uWS::Hub h;
  PID pid(1.5, 0.0007, 0.0006);    // After training from pid(0.1, 0.0, 0.0) using below tuning settings
  Tune tune(0.03, 0.0001, 0.0001);

  h.onMessage([&pid, &tune, &step](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) 
  {
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
      if (s != "") 
      {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") 
        {
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
   
          double steering_value = pid.Update(cte);
          if (TUNE_CONTROLLER)
          { 
            step++;
            if (step == FIRST_SAMPLE_STEP)
            {
              pid.total_error = 0.0;
            }
            if (step == LAST_SAMPLE_STEP)
            {
              step = 0;
              tune.Update(pid);
              std::cout << "P: " << pid.Kp << "   ";
              std::cout << "I: " << pid.Ki << "   ";
              std::cout << "D: " << pid.Kd << std::endl;
              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
          }

          json msgJson;
          msgJson["steering_angle"] = steering_value;
          msgJson["throttle"] = 0.15; 
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } 
      else 
      {
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) 
  {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      res->end(nullptr, 0);
    }
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) 
  {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
