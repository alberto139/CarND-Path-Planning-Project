#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

using namespace std;

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
  double max_s = 6945.554;

 

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

   // starting lane
  int lane = 1;
  // reference target velocity
  double ref_vel = 0; // mph

  h.onMessage([&ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          


          // Decide Trajectory via a cost function
          // Possible trajectories

            // 1. Stay on current lane, maintain speed
            //if (car[0][5] )
            // 2. Stay on current lane, slow down
            // 3. Stay on current lane, speed up
            // 4. Change lane left, keep speed
            // 5. Change lane left, slow down
            // 6. Change lane left, speed up
            // 7. Change lane right, keep speed
            // 8. Change lane right, slow down
            // 9. Change lane right, speed up

          // chose best lane
          double best_cost = 99.9;
          double best_lane = 6;
          double costs [3] = {0,0,0};
          for (int i=0; i<3; i++)
          {
            double cost = 0;
            double temp_d = i;

            for (auto car = sensor_fusion.begin(); car < sensor_fusion.end(); car ++)
            {
            double other_car_d = car[0][6];
            double other_car_s = car[0][5];
            double other_car_x_vel = car[0][1];
            double other_car_y_vel = car[0][1];
            
              if ((other_car_d - 2 < ((temp_d*4) + 2) && other_car_d  + 2 > ((temp_d*4) + 2))  
                  && (other_car_s - car_s <= 50) && (other_car_s - car_s > -10))
                  {
                    cost += 1;
                    costs[i] += 1;
                  }
            }
            if (cost < best_cost)
            {
              best_cost = cost;
              best_lane = i;
            }
            cout << "lane " << i << " cost: " << cost << endl;
            
          }
          
          
          if (costs[1] == 0)
          {
            best_lane = 1;
          }

          cout << costs[1] << endl;
          cout << "best_lane: " << best_lane << endl;
          lane =  best_lane;
          cout << lane << endl;


          //["sensor_fusion"] A 2d vector of cars and then that car's 
          // [car's unique ID, car's x position in map coordinates, 
          // car's y position in map coordinates, car's x velocity in m/s, 
          // car's y velocity in m/s, car's s position in frenet coordinates, 
          // car's d position in frenet coordinates
          bool slow_down = 0;
          for (auto car = sensor_fusion.begin(); car < sensor_fusion.end(); car ++)
          {
            double other_car_d = car[0][6];
            double other_car_s = car[0][5];
            double other_car_x_vel = car[0][1];
            double other_car_y_vel = car[0][1];

            

            // There are cars close to me
            if ((other_car_d - 2 < car_d && other_car_d  + 2 > car_d)  
                && (other_car_s - car_s <= 30) && (other_car_s - car_s > 0))
            {
              cout << "---------------------------------" << endl;
              cout << (other_car_s - car_s) << endl;
              cout << (other_car_s - car_s <= 20) << endl;
              cout << "car ID: " << car[0][0] << endl;
              cout << "car X: " << car[0][1] << endl;
              cout << "car Y: " << car[0][2] << endl;
              cout << "car X Vel: " << car[0][3] << endl;
              cout << "car Y Vel: " << car[0][4] << endl;
              cout << "car S: " << other_car_s << endl;
              cout << "MYcar S: " << car_s << endl;
              cout << "car D: " << other_car_d << endl;
              cout << "MYcar D: " << car_d << endl;

              // if they are slower than me slow down
         
              slow_down = 1;
              if (ref_vel > 5)
              {
              ref_vel -= 1;
              }
            }
          }
          if (slow_down == 0 && ref_vel < 47){
            ref_vel +=1;
            cout << "speeding up" << endl;
          }
        



          /////////////////////////////// LANE FOLLOWING /////////////////////////////

          // create a list of widely spaced waypoints
          // these will me used to make a spline
          int prev_size = previous_path_x.size();
          vector<double> pts_x;
          vector<double> pts_y;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // Create the points for the spline
          if (prev_size < 2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            pts_x.push_back(prev_car_x);
            pts_y.push_back(prev_car_y);
            pts_x.push_back(car_x);
            pts_y.push_back(car_y);
          }
          else
          {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];

            // Calculate the previous angle of the car
            ref_yaw = atan2(ref_y -  ref_y_prev, ref_x - ref_x_prev);


            pts_x.push_back(ref_x_prev);
            pts_x.push_back(ref_x);
            pts_y.push_back(ref_y_prev);
            pts_y.push_back(ref_y);

          }

          //In Frenet add evenly 30m spaced points ahead of the starting reference

          // Get 3 waiy points ahead of the starting reference
          //double buffer = 30;
          double distance_buffer = 50;
          vector<double> next_wp0 = getXY(car_s+distance_buffer,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+distance_buffer*2,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+distance_buffer*3,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp3 = getXY(car_s+distance_buffer*4,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          pts_x.push_back(next_wp0[0]);
          pts_x.push_back(next_wp1[0]);
          pts_x.push_back(next_wp2[0]);
          pts_x.push_back(next_wp3[0]);

          pts_y.push_back(next_wp0[1]);
          pts_y.push_back(next_wp1[1]);
          pts_y.push_back(next_wp2[1]);
          pts_y.push_back(next_wp3[1]);

          // rotate to match the car reference
          for (int i=0; i<pts_x.size(); i++)
          {
            //shfft car reference angle to 0 degrees
            double shift_x = pts_x[i] - ref_x;
            double shift_y = pts_y[i] - ref_y;

            pts_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            pts_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          // create a spline
          tk::spline s;

          // set the points for the spline
          s.set_points(pts_x, pts_y);

          // define the points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // start with all of the previous path points from last time
          for(int i=0; i<previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // calculate how to break up the spline points so that we travel at our desired reference velocity
          double target_x = 70; // largest x in the spline
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
          double x_add_on = 0;


          // fill up the rest of our path planner 
          for (int i=0; i <= 70-previous_path_x.size(); i++)
          {
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // go back to global coordinates form car coordinates
            x_point = (x_ref*cos(ref_yaw)- y_ref * sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }

     
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          /////////////////////////////// END LANE FOLLOWING /////////////////////////////
          

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

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