/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 				Aaron Brown
 **********************************************/

/**
 * @file main.cpp
 **/

#include <string>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <iostream>
#include <fstream>
#include <typeinfo>

#include "json.hpp"
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include "Eigen/QR"
#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "utils.h"
#include "pid_controller.h"

#include <limits>
#include <iostream>
#include <fstream>
#include <uWS/uWS.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <time.h>
#include <tuple>

using namespace std;
using json = nlohmann::json;

#define _USE_MATH_DEFINES
#define PI 3.14159265

string hasData(string s) {
  auto found_null = s.find("null");
    auto b1 = s.find_first_of("{");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
      return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
      return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double angle_between_points(double x1, double y1, double x2, double y2){
  return atan2(y2-y1, x2-x1);
}

double yaw_correction(double yaw) {
    
  	if(yaw < -PI){yaw += 2 * PI;}
    if(yaw > PI){yaw -= 2 * M_PI;}
    
    return yaw;
}


BehaviorPlannerFSM behavior_planner(
      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);

// Decalre and initialized the Motion Planner and all its class requirements
MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

bool have_obst = false;
vector<State> obstacles;

void path_planner(vector<double>& x_points, vector<double>& y_points, vector<double>& v_points, double yaw, double velocity, State goal, bool is_junction, string tl_state, vector< vector<double> >& spirals_x, vector< vector<double> >& spirals_y, vector< vector<double> >& spirals_v, vector<int>& best_spirals){

  State ego_state;

  ego_state.location.x = x_points[x_points.size()-1];
  ego_state.location.y = y_points[y_points.size()-1];
  ego_state.velocity.x = velocity;

  if( x_points.size() > 1 ){
  	ego_state.rotation.yaw = angle_between_points(x_points[x_points.size()-2], y_points[y_points.size()-2], x_points[x_points.size()-1], y_points[y_points.size()-1]);
  	ego_state.velocity.x = v_points[v_points.size()-1];
  	if(velocity < 0.01)
  		ego_state.rotation.yaw = yaw;

  }

  Maneuver behavior = behavior_planner.get_active_maneuver();

  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);

  if(behavior == STOPPED){

  	int max_points = 20;
  	double point_x = x_points[x_points.size()-1];
  	double point_y = y_points[x_points.size()-1];
  	while( x_points.size() < max_points ){
  	  x_points.push_back(point_x);
  	  y_points.push_back(point_y);
  	  v_points.push_back(0);

  	}
  	return;
  }

  auto goal_set = motion_planner.generate_offset_goals(goal);

  auto spirals = motion_planner.generate_spirals(ego_state, goal_set);

  auto desired_speed = utils::magnitude(goal.velocity);

  State lead_car_state;  // = to the vehicle ahead...

  if(spirals.size() == 0){
  	cout << "Error: No spirals generated " << endl;
  	return;
  }

  for(int i = 0; i < spirals.size(); i++){

    auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory( spirals[i], desired_speed, ego_state,
                                                                                    lead_car_state, behavior);

    vector<double> spiral_x;
    vector<double> spiral_y;
    vector<double> spiral_v;
    for(int j = 0; j < trajectory.size(); j++){
      double point_x = trajectory[j].path_point.x;
      double point_y = trajectory[j].path_point.y;
      double velocity = trajectory[j].v;
      spiral_x.push_back(point_x);
      spiral_y.push_back(point_y);
      spiral_v.push_back(velocity);
    }

    spirals_x.push_back(spiral_x);
    spirals_y.push_back(spiral_y);
    spirals_v.push_back(spiral_v);

  }

  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);
  int best_spiral_idx = -1;

  if(best_spirals.size() > 0)
  	best_spiral_idx = best_spirals[best_spirals.size()-1];

  int index = 0;
  int max_points = 20;
  int add_points = spirals_x[best_spiral_idx].size();
  while( x_points.size() < max_points && index < add_points ){
    double point_x = spirals_x[best_spiral_idx][index];
    double point_y = spirals_y[best_spiral_idx][index];
    double velocity = spirals_v[best_spiral_idx][index];
    index++;
    x_points.push_back(point_x);
    y_points.push_back(point_y);
    v_points.push_back(velocity);
  }


}

void set_obst(vector<double> x_points, vector<double> y_points, vector<State>& obstacles, bool& obst_flag){

	for( int i = 0; i < x_points.size(); i++){
		State obstacle;
		obstacle.location.x = x_points[i];
		obstacle.location.y = y_points[i];
		obstacles.push_back(obstacle);
	}
	obst_flag = true;
}

tuple<double, int> cte_calculation(vector<double>& waypoints_x, vector<double>& waypoints_y, double& x, double& y, double& yaw, bool debugMode=false){
  /**
    Computes the cross track error

    @param waypoints_x x-axis vector from waypoints coming from path planner 
    @param waypoints_y y-axis vector from waypoints coming from path planner 
    @param x  current location of ego car.
    @param x  current location of ego car.
    @return tuple (cross track error_calculation+delta_angle,segment of the waypoints that matched the current location)
    
  */
  size_t size_waypoints = waypoints_x.size()-1;
  if(size_waypoints==0){return {0.0, 100};}
  int segment = 0;
  double angle_factor = 1.0;
  while (segment<size_waypoints) 
  {
    double x_2 = waypoints_x[segment+1];
    double y_2 = waypoints_y[segment+1];
    double x_1 = waypoints_x[segment];
    double y_1 = waypoints_y[segment];
    double Rx = x - x_1;
    double Ry = y - y_1;
    double delta_x = x_2 - x_1;
    double delta_y = y_2 - y_1;
    double delta_angle = atan2(delta_y, delta_x);
    double delta_x_squared = delta_x * delta_x;
    double delta_y_squared = delta_y * delta_y;
    double u = (Rx*delta_x+Ry*delta_y)/sqrt(delta_x_squared+delta_y_squared);
    

    
    if (u > 1.0)
    {
      if(segment == size_waypoints-1){
        //calculating the cross track error for last segment 
        double cte = (Ry*delta_x - Rx*delta_y)/sqrt(delta_x_squared+delta_y_squared);
        //calculating the delta angle between the yaw and the waypoint values for last segment
      	double delta_alpha = angle_factor*yaw_correction(yaw - delta_angle)/(PI);
      	if(cte!=cte){cte=0.0;} //returns a cte zero in case cte is NaN
        return {-(cte+delta_alpha), segment};
		
      }
      segment ++;
      
    }else
    {

      //calculating the cross track error for current segment
      double cte = (Ry*delta_x - Rx*delta_y)/sqrt(delta_x_squared+delta_y_squared);
      //calculating the delta angle between the yaw and the waypoint values for current segment
      double delta_alpha = angle_factor*yaw_correction(yaw - delta_angle)/(PI);
      if(cte!=cte){cte=0.0;} //returns a cte zero in case cte is NaN
      // The following three lines are for visualizing on console the current status of the error
      if(debugMode){
        cout << "*********delta alpha normalized:*******: "<<  delta_alpha << endl;
        cout << "*********delta alpha ******* : "<<  (yaw - delta_angle)*180/PI << endl;
        cout << "********* CTE *******: "<<  cte << endl;
      }
      return {-(cte+delta_alpha), segment};
      
      
    }
  } 
 
}


int main ()
{
  cout << "starting server" << endl;
  uWS::Hub h;

  double new_delta_time;
  int i = 0;

  fstream file_steer;
  file_steer.open("steer_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_steer.close();
  fstream file_throttle;
  file_throttle.open("throttle_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_throttle.close();

  time_t prev_timer;
  time_t timer;
  time(&prev_timer);

  // initialize pid steer
  /**
  * TODO (Step 1): create pid (pid_steer) for steer command and initialize values
  **/
  PID pid_steer = PID();
  //pid_steer.Init(0.15, 0.00, 0.0, 1.2, -1.2);
  //pid_steer.Init(0.25, 0.00, 0.0, 1.2, -1.2);
  //pid_steer.Init(0.25, 0.01, 0.0, 1.2, -1.2);
  //pid_steer.Init(0.25, 0.01, 0.5, 1.2, -1.2);
  //pid_steer.Init(0.25, 0.001, 0.2, 1.2, -1.2);
  //pid_steer.Init(0.25, 0.002, 0.2, 1.2, -1.2);
  //pid_steer.Init(0.25, 0.002, 0.2, 1.2, -1.2);
  pid_steer.Init(0.25, 0.001, 0.3, 1.2, -1.2);
  // initialize pid throttle
  /**
  * TODO (Step 1): create pid (pid_throttle) for throttle command and initialize values
  **/
  PID pid_throttle = PID();
  //pid_throttle.Init(0.1, 0.01, 0.05, 1.0, -1.0);
  //pid_throttle.Init(0.15, 0.01, 0.05, 1.0, -1.0);
  //pid_throttle.Init(0.25, 0.01, 0.05, 1.0, -1.0);
  //pid_throttle.Init(0.25, 0.005, 0.05, 1.0, -1.0);
  //pid_throttle.Init(0.25, 0.005, 0.03, 1.0, -1.0);
  pid_throttle.Init(0.25, 0.005, 0.01, 1.0, -1.0);


  h.onMessage([&pid_steer, &pid_throttle, &new_delta_time, &timer, &prev_timer, &i, &prev_timer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
        auto s = hasData(data);

        if (s != "") {

          auto data = json::parse(s);

          // create file to save values
          fstream file_steer;
          file_steer.open("steer_pid_data.txt");
          fstream file_throttle;
          file_throttle.open("throttle_pid_data.txt");

          vector<double> x_points = data["traj_x"];
          vector<double> y_points = data["traj_y"];
          vector<double> v_points = data["traj_v"];
          double yaw = data["yaw"];
          double velocity = data["velocity"];
          double sim_time = data["time"];
          double waypoint_x = data["waypoint_x"];
          double waypoint_y = data["waypoint_y"];
          double waypoint_t = data["waypoint_t"];
          bool is_junction = data["waypoint_j"];
          string tl_state = data["tl_state"];

          double x_position = data["location_x"];
          double y_position = data["location_y"];
          double z_position = data["location_z"];

          if(!have_obst){
          	vector<double> x_obst = data["obst_x"];
          	vector<double> y_obst = data["obst_y"];
          	set_obst(x_obst, y_obst, obstacles, have_obst);
          }

          State goal;
          goal.location.x = waypoint_x;
          goal.location.y = waypoint_y;
          goal.rotation.yaw = waypoint_t;

          vector< vector<double> > spirals_x;
          vector< vector<double> > spirals_y;
          vector< vector<double> > spirals_v;
          vector<int> best_spirals;

          path_planner(x_points, y_points, v_points, yaw, velocity, goal, is_junction, tl_state, spirals_x, spirals_y, spirals_v, best_spirals);

          // Save time and compute delta time
          time(&timer);
          new_delta_time = difftime(timer, prev_timer);
          prev_timer = timer;

          ////////////////////////////////////////
          // Steering control
          ////////////////////////////////////////

          /**
          * TODO (step 3): uncomment these lines
          **/
          // Update the delta time with the previous command
          pid_steer.UpdateDeltaTime(new_delta_time);

          // Compute steer error
          
          double error_steer;
          double steer_output;
		  int segment;
          /**
          * TODO (step 3): compute the steer error (error_steer) from the position and the desired trajectory
          **/
          
          
          
          tie(error_steer, segment) = cte_calculation(x_points, y_points, x_position, y_position, yaw);
          // These lines help checking the status of the vehicle from the console
          cout << "*********waypoints: *******: "<< x_points[segment+1]<<","<< y_points[segment+1] << endl;
          cout << "*********current position*******: "<< x_position <<"," << y_position << endl;
		      cout << "********* segment: *******: "<< segment << endl;
          cout << "*********steer_error_calculation*******: "<< error_steer << endl;
          cout << "*********yaw:*******: "<< yaw*180/PI << endl;
          cout<< "****** dt: ********"<<pid_steer.dt<<endl;
          cout<< "****** time: ********"<<timer<<endl;
		  
          /**
          * TODO (step 3): uncomment these lines
          **/
           // Compute control to apply
           pid_steer.UpdateError(error_steer);
           steer_output = pid_steer.TotalError();

           // Save data
           file_steer.seekg(std::ios::beg);
           for(int j=0; j < i - 1; ++j) {
               file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
           }
           file_steer  << i ;
           file_steer  << " " << error_steer;
           file_steer  << " " << steer_output;
           file_steer  << " " << x_points[segment+1]; 
           file_steer  << " " << y_points[segment+1];
           file_steer  << " " << x_position;
           file_steer  << " " << y_position << endl;

          ////////////////////////////////////////
          // Throttle control
          ////////////////////////////////////////

          /**
          * TODO (step 2): uncomment these lines
          **/
           // Update the delta time with the previous command
          pid_throttle.UpdateDeltaTime(new_delta_time);

          // Compute error of speed
          double error_throttle;
          /**
          * TODO (step 2): compute the throttle error (error_throttle) from the position and the desired speed
          **/
          // modify the following line for step 2
          
          //error_throttle = velocity - v_points[v_points.size()-1];
         
          error_throttle = v_points[segment+1] - velocity;
          
          

          double throttle_output;
          double brake_output;

          /**
          * TODO (step 2): uncomment these lines
          **/
          // Compute control to apply
          pid_throttle.UpdateError(error_throttle);
          double throttle = pid_throttle.TotalError();
          
          
          
          // reset integrators whenever there is no match with a waypoint line
          // for throttle we reduce it slowly instead of resetting it completely to avoid sudden stops
          if (segment >= v_points.size()-2)
          {
            
            pid_throttle.i_error -= 5.0;
            pid_steer.i_error = 0.0;
            
          }
          if(spirals_x.size()==0 || spirals_y.size()==0 || segment==100){
          	throttle = -1.0;
            
          }
          // Adapt the negative throttle to break
          if (throttle > 0.0) {
            throttle_output = throttle;
            brake_output = 0;
          } else {
            throttle_output = 0;
            pid_throttle.i_error = 0.0;
            pid_steer.i_error = 0.0;
            brake_output = -throttle;
          }

          // Save data
          file_throttle.seekg(std::ios::beg);
          for(int j=0; j < i - 1; ++j){
              file_throttle.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
          }
          file_throttle  << i ;
          file_throttle  << " " << error_throttle;
          file_throttle  << " " << brake_output;
          file_throttle  << " " << throttle_output << endl;


          // Send control
          json msgJson;
          msgJson["brake"] = brake_output;
          msgJson["throttle"] = throttle_output;
          msgJson["steer"] = steer_output;

          msgJson["trajectory_x"] = x_points;
          msgJson["trajectory_y"] = y_points;
          msgJson["trajectory_v"] = v_points;
          msgJson["spirals_x"] = spirals_x;
          msgJson["spirals_y"] = spirals_y;
          msgJson["spirals_v"] = spirals_v;
          msgJson["spiral_idx"] = best_spirals;
          msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();

          //  min point threshold before doing the update
          // for high update rate use 19 for slow update rate use 4
          msgJson["update_point_thresh"] = 16;

          auto msg = msgJson.dump();

          i = i + 1;
          file_steer.close();
          file_throttle.close();

      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

    }

  });


  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
      cout << "Connected!!!" << endl;
    });


  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
      ws.close();
      cout << "Disconnected" << endl;
    });

  int port = 4567;
  if (h.listen("0.0.0.0", port))
    {
      cout << "Listening to port " << port << endl;
      h.run();
    }
  else
    {
      cerr << "Failed to listen to port" << endl;
      return -1;
    }


}
