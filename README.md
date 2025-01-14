# Control and Trajectory Tracking for Autonomous Vehicle

# Proportional-Integral-Derivative (PID)

In this project, I designed 2 PID controllers to perform vehicle trajectory tracking. Given a trajectory as an array of locations, and a simulation environment, a PID controller algorithm was developed, and then used for controlling the steering and the throttle of the vehicle. Finally it was tested on the CARLA simulator used in the industry for assessing its performance.


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

### Environment Tips:

- When testing the c++ code, restart the Carla simulator to remove the former car from the simulation.
- If the simulation freezes on the desktop mode but is still running on the terminal, close the desktop and restart it.


# Project implementation description

I started by developing the PID controller. Hence, I defined a proportional error (ep), derivate error (ed) and integral error (ie). After that the I set the gains for each of the errors. In the end the PID control output was given by the following equation:

```
u = kp * ep + ki*ei + kd * ed
```

where: 

* kp, ki, kd are the gains for the proportional, integral and differential terms.
* ep is the proportional error that is the same as the current error
* ei is the integral of the error that was defined by the change in time (dt) * current error as this is a discrete system.  
* ed is the differential error that is defined as delta_error/dt


After implementing these two errors I had to define how the error for both steering and throttle will be computed. I started with the steering as it was more challenging. 

## Steering error definition. 

I designed the steering error based on the concept of cross track error (CTE) and the angle difference between the yaw and the angle generated by the waypoints segments. 

To implement the CTE I did some drawings which I will use to also illustrate this explanation. 

<center><img src = "./project/pid_controller/screenshot/CTE_diagram.png"/></center>

As we can see above the vehicle is presented with a set of waypoints that it has to follow. For this project the vector size of waypoints is 20. Hence, a vector `[(x1,y1),(x2,y2),...(x20,y20)]` of waypoint pairs are presented. 

We use this information to first calculate the projection of our current vehicle position (depicted in the diagram above as (x,y)) to the segments between each waypoints. i.e. I assume a vector between waypoints and then another vector from the beginning of the starting point of the waypoint segment to the current position of our ego car. We finally normalize this projection by using the unit vector of the waypoint segment (the unit vector of the purple vector) 

This projection helped me determined when to change of segment. If the projection exceeds the magnitude of 1 that means that I must use the next segment and so on. 

However, the real CTE value was calculated by projecting our vector `C` to the perpendicular unit vector of the waypoint segment (depicted in green in the diagram above). To achieve this I used the dot product property since two vectors that are perpendicular to each other have a dot product of zero. The whole derivation is shown below: 

<center><img src = "./project/pid_controller/screenshot/CTE_derivation.png"/></center>

In the end the final equation for the CTE is as follows:

```
CTE = (Ry*delta_x - Rx * delta_y)/sqrt(delta_x^2+delta_y^2)
```

On the other hand, the last diagram in the image above depicts the angle difference between the yaw and the waypoint segment vector. For this I used atan2 instead of atan since it avoids division by zero and has a range between [-pi, pi]. Thus the angle difference is defined as follows:

```
delta_theta = yaw - atan2(delta_y,delta_x)
```
Finally we add this two errors to define the final steering error as follows:

```
steering error = CTE + angle_factor * delta_theta
```
where `angle_factor` is just a factor that I added to control how much influence delta_theta will have in the steering error. 

The steering error was designed this way as the delta_theta accounts for quick changes in the winding path whereas the CTE is more stable and measure how far the ego car is from the waypoint segment. In the end, this combination proved to be successful for our use case.

## Throttle error

For the throttle error the implementation was pretty straightforward as I defined as the difference between the current velocity and the velocity of the last waypoint of the current segment computed by the CTE explained in the steering error section.

```
Throttle error = Velocity[segment] - Current_velocity
```

## PID calibration

To calibrate the PID I followed a basic approach used in control theory which is basically trial and error. However, the trial and error approach I followed has an algorithm to get some set of gains that stabilize the system quicker. This algorithm is as follows:

* Set to zero integral and derivative gains and increase the proportional gain until the system starts to reach the request with some oscillations. 
* Decrease the proportional until oscillations are almost decreased. At this point I mostly get some stationary error.
* To get rid of the stationary error I gradually increase the integral gain.
* Finally if needed I increase the derivative gain especially if the system has sudden changes of requests. For instance, in the steering, this happened when we turned to the right in the first traffic light. However, I almost always leave the derivative gain smaller than the other gains as it is very sensitive to noisy signals which we always have in electric/electronic systems.

## Results

To check the results I developed a new plot that helped me a lot in fine-tuning the controllers. The two plots provided are very good to see how the error is decreasing. However, in control is also useful to see the requested signal vs the actual signal of the system. This way you can clearly see for instance if there are overshoots, or oscillations in the system. 


### 1st functional version

The results of the 1st version of a functional PID calibration are the following:

<center><img src = "./project/pid_controller/screenshot/request_pos_VS_actual_pos_v1.png"/></center>
<center><span class="caption"> waypoint request position trajectory (blue) VS  actual position of the car (orange) </span></center>

<center><img src = "./project/pid_controller/screenshot/steering_plot_v1.png"/></center>
<center><span class="caption"> Error steering (blue) VS Steering control output (orange) </span></center>

<center><img src = "./project/pid_controller/screenshot/throttle_v1.png"/></center>
<center><span class="caption"> Error throttle (blue) VS Throttle control output (green) VS Brake output (orange) </span></center>

### Challengues of the implementation

After some quick fine-tunning this 1st version was pretty good as you can see above. However, I found that there were some bugs in the implementation of perhaps the planner as during this test case I got no splines in a particular section, which resulted in a bad request that you can see at the end of the trajectory in the 1st plot. Additional to this, these are some other findings from the 1st version:

* No splines were found leading to not having a request for the car and consequently the ego car performed unexpected maneuvers. You can see an example of my command line while debugging this issue:

<center><img src = "./project/pid_controller/screenshot/no_spirals_error.png"/></center>
<center><span class="caption"> No spirals generated error </span></center>

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**Solution:** Whenever no spline is given I immediatly stop the car to wait until a spline is generated. 

* Sometimes my CTE calculation does not matched any segment of the waypoint. hence the car sticked with the same steering angle for a period, which I found very risky. 

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**Solution:** I implemented a section that takes into account the last segment even if it does not match 100% to it. Since it didn't match to a segment I also decrease the speed of the car when this happens as we are not sure about the CTE calculation. 

* Sometimes a change in time `dt = 0` was obtained which yield a division by zero in the differential term of the PID controller. 

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**Solution:** To avoid this I added a sanity check when dt = 0 in the differential term of my PID implementation.

* I found that the throttle controller stopped abruptly when I wait for splines or if the CTE does not find a segment and I thought this could make feel the user a little bit dizzy. I realidzed this was due to the acumulation of the integral error. 

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**Solution:** To avoid this I decided to reset the integral error everytime I stopped the car, this includes also the cases where I had to stop the car when no spline is generated by the planner. For the throttle though I decided to decrease by certain ratio the integral error instead of directly resetting the error as this way the velocity does not get changed abruptly.


### Final functional version

After implementing all the solutions to the problems found in the 1st version I finu-tune the controller a little bit more. The results are the following:

<center><img src = "./project/pid_controller/screenshot/request_pos_VS_actual_pos_verFinal.png"/></center>
<center><span class="caption"> waypoint request position trajectory (blue) VS  actual position of the car (orange) </span></center>

<center><img src = "./project/pid_controller/screenshot/steering_plot_verFinal.png"/></center>
<center><span class="caption"> Error steering (blue) VS Steering control output (orange) </span></center>

<center><img src = "./project/pid_controller/screenshot/throttle_verFinal.png"/></center>
<center><span class="caption"> Error throttle (blue) VS Throttle control output (green) VS Brake output (orange) </span></center>

You can see that the main difference is in the throttle controller where the error does not change that harshly as in the 1st version. This yielded a more comfortable velocity for the user. Also, bugs do not crashed our implememtation during this test case. 


### Lessons learned from the PID controller

**How each part of the PID affects the control command?**

In summary while tunning the PID controllers I found the following:

* Proportional helps reaching the request either of steering or throttle, but with a stationary error if the gain is just right or with oscillations if it is too high. 
* Integral solves the stationary error problem. However, resetting the integral error whenever the system is stopped is required to avoid acumulation of error. 
* Differential or derivative term helps reacting to changes in the request. However, we should keep it small as otherwise noisy signals can make it unstable. 


**Is it possible to automate the fine-tuning process of the PID parameters?**

In this case, it was not that difficult to fine-tune the model. Hence, a trial and error approach was enough. However, I can see the benefit of using some algorithm to fine-tune the controller's parametrs such as **twiddle**. This approach is like a brute force algorithm that tries some combinations of kp,kd,ki and gradually modify them, to finally keep the combination that produces the least error of all. 

Another way to fine-tune a PID whenever you want to use it for an online task, is an adaptive PID controller. 
One very interesting approach is the self-adaptive Neural Network PID controller.  

<center><img src = "./project/pid_controller/screenshot/self_adaptive_PIDcontroller.jpeg"/></center>
<center><span class="caption"> Diagram taken from <cite><a href="https://www.osapublishing.org/jocn/home.cfm">Here</a></cite> </span></center>


As you can see above we basically feed the errors to the network, sometimes they can be the proportional, integral, and differential errors or errors in the past (as in a time delay network shown in the figure above) and it learns all the weights to output a certain kp, ki and kd gains to reduce the error of the task. This is a very interesting approach that we could also implement in this project as future work.  


**Pros and Cons of a model free controller such as a PID controller?**

#### Pros 
* PID is a model free controller because it does not require the model of the system (in this case the car) to design the controller. Many times it is very difficult to get the system's model. Hence having a model free controller is handy. Especially when the system's model is not given or impossible to obtain.  

* It can be computationally cheaper and also easier to implement.

* Model-based controllers can lead to implementation not well suited for the task if the model is too abstract. This problem does not affect model-free controllers.


#### Cons

* You might need to tune the model free model differently for edge cases which makes tuning more dangerous. Whereas in model-based algorithms you sort of have an idea already of what will happen.

* If a great model is developed for certain application, the model-based controller will most likely outperform a model free model. 

* Sometimes it takes a lot of effort to find the best parameters for the model free model.

* There is an additional CON for traditional PID controllers (this does not necessarily apply for all model-free models), but PID controllers are designed for controlling linear systems, they do not work as great for non-linear systems (unless we add some other features to them or we linearize the system). Whereas model-based controllers can simply use a non-linear model and get very good results. For instance, in terms of the vehicle, we could use the bicycle model, which is a very popular non-linear model that captures the dynamics of the vehicle in a simplified way. 


## Conclusion

In conclusion this was a very interesting problem that served as a good example and practice on how a simple PID controller can be used to follow a path from a trajectory given by the path planner component. We learned how each term of the PID affect in the response to trajectory waypoint requests and to some perturbation to the system such as paths that are very sharp. Overall this project is a great point to begin inmmersing in the world of controll systems. 

The final result of my implementation can be found in the following [video](./project/pid_controller/screenshot/PID_controller_in_action.mp4)

## Future work

* As future work, we can implement the so call self-adaptive PID controller to see if the PID parameters that the network finds are better than the gains I found by trial and error. 

* Try a model-based controller with a non-linear model of the car, perhaps using the bicycle model. A model that comes to my mind right now is an MPC controller since this controller allows us to set constraints that we can use for tunning a very smooth trajectory follow and consequently comfort of the user (for instance constraints in acceleration or constraints in jerk). 







