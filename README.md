# Path-Planning
**Self-Driving Car Engineer Nanodegree Program**

### Introduction
In this project, the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. We are provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The goal is to drive close to 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s<sup>2</sup> and jerk that is greater than 10 m/s<sup>3</sup>.

The project consists of following files in the `src` directory: 

* `main.cpp` :  The main file which includes the script to drive the car.
* `map.cpp` and `map.h` : The `Map` class consists of functions to prepare a smooth waypoint trajectory using the `spline` function. Also includes functions to convert from Cartesian to Frenet coordinates and vice-versa. 
* `vehicle.cpp` and `vehicle.h` : Consists of functions pertaining to traffic vehicles included under class `Vehicle` as well as functions pertaining to ego vehicle itself included under derived class `EgoVehicle`. 
* `traffic.cpp` and `traffic.h` : Includes two classes `Lane` and `Traffic`. `Lane` holds vector of all traffic vehicles at any given moment in that lane. The purpose of `Traffic` is to fill up the `Lane` object with vehicles using sensor data. It also collects information about nearby traffic with respect to ego vehicle and prepapres possible target states. 
* `trajectory.cpp` and `trajectory.h` : Consists of two classes : `Trajectory` and `TrjPlanner`. `Trajectory` stores the trajectory of ego vehicle as well as the cost associated with that trajectory. `TrjPlanner` contains functions to plan the trajectory given the boundary conditions and find the best trajectory.  
* `utils.cpp` and `utils.h` : Includes utility functions and classes, most importantly a function to plan `s` trajectory. 

### Description
The basic skelton of path planning is implemented in `main.cpp`. To predict future trajectory of ego vehicle, we use 50 points from previous trajectory prediction (line 91) to obtain a smooth trajectory. The starting point for the ego vehicle state is therefore the state at the end of last predicted trajectory (lines 98-99). Next, we read sensor fusion data (lines 107-124) and fill the lanes with traffic (lines 125). Note that for consistency, the traffic vehicle state must be calculated at the same time as the starting state of ego vehicle. To achieve this, we run a constant speed simulation, assuming the traffic stays in the same lane (line 116). Next, we plan possible target states for the ego vehicle given the initial state (line 132). Depending on the nearby traffic, best trajectory is selected (line 138). This is the basic overview of our algorithm. Below we give some implementation details :

** Smooth waypoints generation ** 
Every cycle, we generate smooth waypoints using `spline` in the vicinty of ego vehicle using the coarse waypoints provided (see `Map.prepare` function on line 117 of `map.cpp`). This allows us to generate accurate Frenet coordinates (see function `Map.getFrenet` on line 68 of `map.cpp`).  

** Building Targets **
For each lane, we find the nearest traffic vehicle in front and behind the ego vehicle. A lane is marked as `BLOCKED` if there is an obstacle vehicle in the vicinty of ego vehicle (15 m ahead and 8 m behind). Of course for the ego vehicle lane, we only care about the front. A lane is marked `FREE` if there is no vehicle present in a range of 45 m. In all other cases, lane is marked `BUSY` (see function `Lane.analyze_traffic` on line 130 of `traffic.cpp`). 

Once the lanes are marked, we construct target states (function `Traffic.plan_target_candidates` on line 80 of `traffic.cpp`). If the current ego lane is marked `FREE`, we do not attempt to find more targets i.e. we stay in the lane. If the ego lane is `BUSY`,  we attempt lane change if the adjacent lane(s) are either `FREE`. In case adajaccent lane is `BUSY`, it is still a potential target if there is no obstacle vehicle ahead. In all these cases, the target speed `s_dot` is max speed (22.35 m/s ~ 50 mph) minus a buffer speed (0.7 m/s). The target distance `s` in these cases is determined by the planning time which we will discuss later. Until now, we have discussed cases when there is no traffic ahead. In case, there is a vehicle ahead, the target speed is the speed of vehicle ahead minus a buffer speed margin (see line 72 of `traffic.cpp`). The target distance `s` is the obstacle vehicle `s` minus a safe distance we take to be 15 m.  We can also calculate how much distance ego vehicle needs to achieve the target velocity using equations of motion :  &Delta; s = (v<sup>2</sup> - u<sup>2</sup>) / (2 a). If this distance is much less than the target distance, it is too early to slow down, so target speed in this case is kept at the maximum speed (see function `Traffic.plan_target` on line 53 of `traffic.cpp`). The target `d` is taken to be the center of the target lane while target `s_ddot`, `d_dot` and `d_ddot` are all fixed to zero to achieve stable end configuration.

** Trajectory generation ** 
We use a combination of analytical and numerical methods for trajectory generation. For `s`-coordinate trajectory, we rely on analytical methods (see function `s_trajectory` on line 18 of `utils.cpp`). The logic is as follows. If the initial ego vehicle speed is more than target speed, we must slow down. If the ego vehicle is currently accelrating, we need to immediately start decelerating. To do so, we decrease the acceleration by `DA_SIM` = 1.5 m/s<sup>2</sup> in time `DT_SIM` = 0.02 s (see line 40 of `utils.cpp`). This value is chosen to keep jerk within the required limit. On the other hand, if the ego vehicle is already decelerating, we may in fact need to slowly release the brakes so as to attain a stable constant speed configuration. At exactly what point should we start releasing the brakes? This point can be easily calculated analytically using equations of motion. The critical speed at which to start releasing the brakes is `v_relax = s_dot + n * s_dot * DT_SIM   - 0.5 * n * (n-1) * DA_SIM * DT_SIM` where `n = |a / DA_SIM|` is the number of time steps required to reach constant speed configuration (see line 56 of `utils.cpp`). Exactly same logic can be used when target speed is more than the initial speed, one can determine at what point should we start lifting our foot from the gas pedal. This algorithm also gives us the time needed to perform the required maneuver such as following an obstacle vehicle or lane changing behind an obstacle vehicle. If there is no obstacle ahead, we simply generate trajectory until plan time (3.5 secs) minus the duration of previous trajectory.   

For `d`-coordinate trajectory, we use the jerk-minimizing trajectory (see function `TrjPlanner.JMT` on line 216 of `trajectory.cpp`) time duration as determined by the `s`-trajectory. 

** Best Trajectory **
To find the best trajectory, we associate a cost with each trajectory and choose the one with minimum cost (see function `TrjPlanner.best_trajectory` on line 201 of `trajectory.cpp`).
The cost function for trajectories is calculated in the same file in the function `Trajectory.cost` on line 70. Since, many of the restrictions such as maximum speed, acceleration and jerk are already hard coded in the `s`-trajectory generator, there inclusion in the cost function only plays a minor role. The major role is played by the `COLLISION` cost which adds a big penalty for collision with other cars.  To motivate ego vehicle to overtake when possible, we subtract a *reward* term from the cost where the reward is proportional to the distance traveled. Lastly, to prevent ego vehicle from making sharp lane changes, we add a penalty proportional to |&Delta; d / &Delta; s| i.e. lane changes over a small distance is penalized.   

#### Results
Car drives around 46 to 47 mph without any incidents as demonstrated in the video below:

[![Sample](http://img.youtube.com/vi/U_SJpksn0XY/0.jpg)](http://www.youtube.com/watch?v=U_SJpksn0XY "Sample")

It is quite amazing how well the analytically generated `s`-trajectory seems to work.
The only rare incidents occur when the obstacle vehicle suddenly changes lanes in front. 

### Scope for improvement 

* Keep track of obstacles over time for better predictions. Go beyond the assumptions of constant speed and same lane.
* Work on Jerk-minimzing trajectory for `s`. My first attempts at using JMT for `s`-trajectory occasionaly yielded unstable results. 
* Make cost function more robust. It is quite possible that rare cases are not covered yet.

***

**Udacity instructions** (https://github.com/udacity/CarND-Path-Planning-Project) below:  

---- 
  

### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

