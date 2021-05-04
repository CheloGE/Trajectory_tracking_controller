# Control and Trajectory Tracking for Autonomous Vehicle

# Proportional-Integral-Derivative (PID)

In this project, I designed 2 PID controllers to perform vehicle trajectory tracking. Given a trajectory as an array of locations, and a simulation environment, a PID controller algorithm was developed, then implemented for controlling the steering and the throttle of the vehicle. Finally it was tested on the CARLA simulator used in the industry for assessing its performance.


<center>
<div class="row">
  <div class="column">
    <img src="./project/pid_controller/screenshot/driving_car_part1.gif" width="300">
    <img src="./project/pid_controller/screenshot/driving_car_part2.gif" width="300" >
    
  </div>
   <center><span class="caption"> short illustrations to depict the performance of the PID controller  </span></center>
</div>
</center>

### Installation

The project was developed in the Udacity workspcae. Hence, run the following commands to install the starter code in the Udacity Workspace:

place this project inside the Udacity workspace and the run the following steps to compile the project and to run the Carla simulator:

## Run Carla Simulator

Open new window

* `su - student`
// Will say permission denied, ignore and continue
* `cd /opt/carla-simulator/`
* `SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl`

## Compile and Run the Controller

Open new window

* `cd nd013-c6-control-starter/project`
* `./install-ubuntu.sh`
* `cd pid_controller/`
* `rm -rf rpclib`
* `git clone https://github.com/rpclib/rpclib.git`
* `cmake .`
* `make` (This last command compiles the c++ code)

## Testing

To run the simulator on the desktop environment please do the following:

* `cd nd013-c6-control-starter/project`
* `./run_main_pid.sh`
This will silently fail `ctrl + C` to stop
* `./run_main_pid.sh` (again)
Go to desktop mode to see CARLA

If error bind is already in use, or address already being used

* `ps -aux | grep carla`
* `kill id`


## Environment Description

In the previous project we built a path planner for the autonomous vehicle. So in this project we focus on building the steer and throttle controller so that the car follows the trajectory.

You will design and run the a PID controller as described in the previous course.

The most relevant files in this project are:

* [pid_controller.h](./project/pid_controller/pid_controller.h) - In this file we can find the definition of all functions and variables involving the pid_controller
* [pid_controller.cpp](./project/pid_controller/pid_controller.cpp) - In this file we can find the implementation of the PID controller.
* [main.cpp](./project/pid_controller/main.cpp) - In this file we defined all the pipeline to control the steering and throttle and follow the path planner's waypoints. 
* [plot_pid.py](./project/plot_pid.py) - In this file we implemented some code to better visualize the performance of our controllers. It has the following 3 plots:
    * **Position request(waypoints) vs real Vehicles position (in CARLA)**. This plot was extremely useful to debug the code while developing since here we could easily visualize if the controller is overshooting or if it is oscillating. 
    * **Steering plot** - Containing 2 signals Steering error and PID Steering output(AKA control output or control signal)
    * **Throttle plot** - Containing 3 signals Throttle error, PID Throttle output(when the control output is positive) and Brake output (when the control output is negative).

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**Note:** The values of the error and the pid command are saved in [throttle_pid_data.txt](./project/throttle_pid_data.txt) and [steer_pid_data.txt](./project/steer_pid_data.txt)

**Note 2:** Run the plots by using the command (in nd013-c6-control-refresh/project):

```
python3 plot_pid.py
```

You must install a few additional python modules to run these plots: 

```
pip3 install pandas
pip3 install matplotlib
```

* **Useful variables for Throttle controller:**
    - The last point of **v_points** vector contains the velocity computed by the path planner.
    - **velocity** contains the actual velocity.
    - The output of the controller should be inside [-1, 1].

* **Useful variables for Steering controller:**
    - The variable **y_points** and **x_point** gives the desired trajectory planned by the path_planner.
    - **yaw** gives the actual rotational angle of the car.
    - The output of the controller should be inside [-1.2, 1.2].
    - If needed, the position of the car is stored in the variables **x_position**, **y_position** and **z_position**







Answer the following questions:
- Add the plots to your report and explain them (describe what you see)
- What is the effect of the PID according to the plots, how each part of the PID affects the control command?
- How would you design a way to automatically tune the PID parameters?
- PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller?
- (Optional) What would you do to improve the PID controller?


### Tips:

- When testing the c++ code, restart the Carla simulator to remove the former car from the simulation.
- If the simulation freezes on the desktop mode but is still running on the terminal, close the desktop and restart it.


