#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <chrono>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// Define Macros
#define    INVALID_LANE_ID              (-1)
#define    LEFT_LANE_ID                 (0)
#define    MID_LANE_ID                  (1)
#define    RIGHT_LANE_ID                (2)
#define    LANE_WIDTH                   (4)
#define    MAX_LEFT_LANE                (4)
#define    MAX_MID_LANE                 (8)
#define    MAX_RIGHT_LANE               (12)
#define    MAX_LANE_NUM                 (3)
#define    MAX_SPEED                    ((double)(49.5/1)) // 49.5 mph
#define    MAX_ACCELERATION             (0.224) // 0.224 mph (0.1 m/s) per iteration (0.02 seconds) == acceleration of 5 m/s^2
#define    SAFE_DISTANCE                (30) // Safe distance to maintain between our car and other cars
#define    NUM_OF_POINTS                (50)

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// Gets on which Lane is the car given its d
int ExecuteStateMachine(bool isCarAhead, bool isLeftLaneBusy, bool isRightLaneBusy, int current_lane, double &ref_vel)
{
    int new_lane;
    switch(current_lane)
    {
        case LEFT_LANE_ID:
        {
            if(isCarAhead)
            {
                std::cout << "A/L/R"<<isCarAhead<<"/"<<isLeftLaneBusy<<"/"<<isRightLaneBusy<<std::endl;
                // Lane Change is needed OR decrease speed if the next lane is not currently free
                // We are on the left lane so only the middle lane is available
                if(!isRightLaneBusy)
                {
                    // Move to the next lane
                    std::cout << "Move to middle lane1"<<std::endl;
                    std::cout << "A/L/R"<<isCarAhead<<"/"<<isLeftLaneBusy<<"/"<<isRightLaneBusy<<std::endl;
                    ref_vel -= MAX_ACCELERATION;
                    new_lane = MID_LANE_ID;
                }
                else
                {
                    // Next lane is busy decrease the speed to avoid collision
                    ref_vel -= MAX_ACCELERATION;
                    new_lane = current_lane;
                }
            }
            else
            {
                // The road ahead is safe to increase the velocity as long as we do not exceed max speed permitted
                if(ref_vel < MAX_SPEED)
                {
                    ref_vel += MAX_ACCELERATION;
                    new_lane = current_lane;
                }
            }
            break;    
        }
        case MID_LANE_ID:
        {
            std::cout << "Middle Lane"<<std::endl;
            if(isCarAhead)
            {
                std::cout << "Is Car AHEAD "<<isCarAhead<<std::endl;
                // Lane Change is needed OR decrease speed if the next lane is not currently free
                // We are on the middle lane so the left and right lane are available options
                if(!isLeftLaneBusy)
                {
                    // Move to the next lane
                    std::cout << "Move to left lane2"<<std::endl;
                    std::cout << "A/L/R:"<<isCarAhead<<"/"<<isLeftLaneBusy<<"/"<<isRightLaneBusy<<std::endl;
                    ref_vel -= MAX_ACCELERATION;
                    new_lane = LEFT_LANE_ID;
                }
                else if(!isRightLaneBusy)
                {
                    // Move to the next lane
                    std::cout << "Move to right lane3"<<std::endl;
                    std::cout << "A/L/R"<<isCarAhead<<"/"<<isLeftLaneBusy<<"/"<<isRightLaneBusy<<std::endl;
                    ref_vel -= MAX_ACCELERATION;
                    new_lane = RIGHT_LANE_ID;
                }
                else
                {
                    // Next lane is busy decrease the speed to avoid collision
                    std::cout << "Slow down"<<std::endl;
                    ref_vel -= MAX_ACCELERATION;
                    new_lane = current_lane;
                }
            }
            else
            {
                // The road ahead is safe to increase the velocity as long as we do not exceed max speed permitted
                if(ref_vel < MAX_SPEED)
                {
                     //std::cout << "Increase speed"<<std::endl;
                     ref_vel += MAX_ACCELERATION;
                     new_lane = current_lane;
                }
                else
                {
                    //std::cout << "Max speed reached"<<std::endl;
                    new_lane = current_lane;
                }
            }
            break; 
        }
        case RIGHT_LANE_ID:
        {
            if(isCarAhead)
            {
                // Lane Change is needed OR decrease speed if the next lane is not currently free
                // We are on the right lane so only the middle lane is available
                if(!isLeftLaneBusy)
                {
                    // Move to the next lane
                    std::cout << "Move to middle lane4"<<std::endl;
                    std::cout << "A/L/R"<<isCarAhead<<"/"<<isLeftLaneBusy<<"/"<<isRightLaneBusy<<std::endl;
                    ref_vel -= MAX_ACCELERATION;
                    new_lane = MID_LANE_ID;
                }
                else
                {
                    // Next lane is busy decrease the speed to avoid collision
                    ref_vel -= MAX_ACCELERATION;
                    new_lane = current_lane;
                }
            }
            else
            {
                // The road ahead is safe to increase the velocity as long as we do not exceed max speed permitted
                if(ref_vel < MAX_SPEED)
                {
                    ref_vel += MAX_ACCELERATION;
                    new_lane = current_lane;
                }
                else
                {
                     new_lane = current_lane;
                }
            }
            break; 
        }
        default:
        {
            break;
        }
    }
  
  return new_lane;
}

int getCarLane(float d)
{
    int lane_num = INVALID_LANE_ID;
    if( (d > 0) && (d < MAX_LEFT_LANE))
    {
        // The car is in the left lane
        lane_num = LEFT_LANE_ID;
    }
    else if( (d >= MAX_LEFT_LANE) && (d < MAX_MID_LANE) )
    {
        // The car is in the left lane
        lane_num = MID_LANE_ID;
    }
    else if ( (d >= MAX_MID_LANE) && (d< MAX_RIGHT_LANE) )
    {
        // The car is in the left lane
        lane_num = RIGHT_LANE_ID;
    }
    return lane_num;
}

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

  // Start at the middle lane
  int my_lane = MID_LANE_ID;
  double ref_vel = 0.0f; // mph
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &my_lane, &ref_vel]
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

          int prev_size = previous_path_x.size();
          
          // Boolean identifying car surrounding us (ahead/left/right)
          bool is_car_ahead = false;
          bool is_left_lane_busy = false;
          bool is_right_lane_busy = false;
          
          if (prev_size > 2)
          {
            car_s = end_path_s;
          }
          
          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           *   define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          // Stage 1: Analyze other car position
          for(int indx = 0; indx < sensor_fusion.size(); indx++)
          {
              // Extract car parameters (s,d,velocities, etc...)
              double car_vx = sensor_fusion[indx][3];
              double car_vy = sensor_fusion[indx][4];
              double check_car_s = sensor_fusion[indx][5];
              double car_d = sensor_fusion[indx][6];
              double check_speed = sqrt( (car_vx*car_vx) + (car_vy*car_vy) );

              // The Car S after adding previous trajectory
              check_car_s += ((double)prev_size*check_speed*0.02);
              // Locate 
              int other_car_lane = getCarLane(car_d);
              if(INVALID_LANE_ID == other_car_lane)
              {
                  // Unkown lane skip this iteration
                  continue;
              }
            
              // Check surrounding cars (ahead/left/right)
              if(my_lane == other_car_lane)
              {
                  // The other car is in my same lane
                  is_car_ahead |= ( (check_car_s > car_s) && ( (check_car_s - car_s) < SAFE_DISTANCE) );
              }
              else if(other_car_lane - my_lane == -1)
              {
                  // The other car is on my left
                  is_left_lane_busy |= ( ((car_s - SAFE_DISTANCE) < check_car_s) && ((car_s + SAFE_DISTANCE)> check_car_s)  );
              }
             
              else if( other_car_lane - my_lane == 1)
              {
                  // The other car is on my right
                  is_right_lane_busy |= ( ((car_s - SAFE_DISTANCE) < check_car_s) && ((car_s + SAFE_DISTANCE)> check_car_s) );
              }
          }

          // Stage 2 : State Machine Transition Behaviour
          my_lane = ExecuteStateMachine(is_car_ahead, is_left_lane_busy, is_right_lane_busy, my_lane, ref_vel);
          
          // Stage 3: Calculate trajectory to follow
          vector<double> points_x;
          vector<double> points_y;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // if previous size is almost empty, use the car as starting reference
          if (prev_size < 2)
          {
              // Find any 2 points so that the path is tangent to the car
              // use two points that make the path tangent to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);
            
              // Push these 2 points to our vector
              points_x.push_back(prev_car_x);
              points_x.push_back(car_x);
              points_y.push_back(prev_car_y);
              points_y.push_back(car_y);
          }
          else
          {
              // We already have 2 previous points so we can use them
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];
              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2( (ref_y - ref_y_prev), (ref_x - ref_x_prev));
            
              // Push these 2 points to our vector
              points_x.push_back(ref_x_prev);
              points_x.push_back(ref_x);
              points_y.push_back(ref_y_prev);
              points_y.push_back(ref_y);
          }
          
          // In Frenet add points evenly separated by 30m ahead of the starting ref
          for (int point_spacing = 30; point_spacing <= 90; point_spacing += 30)
          {
            vector<double> waypoint = getXY( car_s + point_spacing,
                                            (2+ (4*my_lane)), 
                                            map_waypoints_s,
                                            map_waypoints_x,
                                            map_waypoints_y);
            
            // Push these 3 points to our vector
            points_x.push_back(waypoint[0]);
            points_y.push_back(waypoint[1]);
          }
          
          // Take all the points and translate and rotate to the car's coordinate system
          for (unsigned int indx = 0; indx < points_x.size(); indx++)
          {
               double shift_x = points_x[indx] - ref_x;
               double shift_y = points_y[indx] - ref_y;
            
               points_x[indx] = (shift_x * cos(0 - ref_yaw)) - (shift_y * sin(0 - ref_yaw));
               points_y[indx] = (shift_x * sin(0 - ref_yaw)) + (shift_y * cos(0 - ref_yaw));
          }
          
          // create a spline & set (x,y) points to the spline
          tk::spline spline_car;
          spline_car.set_points(points_x, points_y);
          
          // Use previous path points
          for(int indx = 0; indx < prev_size; indx++)
          {
              next_x_vals.push_back(previous_path_x[indx]);
              next_y_vals.push_back(previous_path_y[indx]); 
          }
          
          
          // Calculate target y after 30 m ahead
          double target_x = 30.0;
          double target_y = spline_car(target_x);
          double target_dist = sqrt( (target_x * target_x) + (target_y * target_y));
          
          double x_car_recent = 0;
          // Complete the rest of the points to 50 after filling it with previous points
          for(int indx = 0; indx < NUM_OF_POINTS - prev_size; indx++)
          {
               double num_of_points = target_dist / (0.02 * ref_vel/2.24);
               double x_point = x_car_recent + (target_x / num_of_points);
               double y_point = spline_car(x_point);
            
               x_car_recent = x_point;
                 
               double x_ref = x_point;
               double y_ref = y_point;
            
              // Rotating points back to normal
               x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
               y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

               x_point += ref_x;
               y_point += ref_y;

               next_x_vals.push_back(x_point);
               next_y_vals.push_back(y_point);
          }
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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