# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

[formulae]: state_update.png

---

## Objective
In this project I implemented Model Predictive Control to drive the car around the track.  
Addidtional requirement : There's a 100 millisecond latency between actuations commands on top of the connection latency.

## Model

This project follows the Kinematic Model. Kinematic models are 
simplifications of dynamic models that ignore tire forces, gravity, and mass. This simplification reduces the accuracy 
of the models, but it also makes them more tractable.

It may be useful to distinguish three significant parts within it: *State*, *Actuators*, *Equations*.

**States**: 

* x: x position
* y: y position
* ψ (psi): orientation (yaw)
* ν: velocity
* cte: cross track error
* eψ : orientation error

**Actuator values**:

* δ (delta): steering angle
* a : throttle (breaking/acceleration).

**Kinematic model standard equations**:

![alt][formulae]

## Implementation

My implementation is slightly refactored version of MPC given in class. It uses https://projects.coin-or.org/Ipopt as a lib for large-scale ​nonlinear optimization. 

#### Coordinate transformations
The optimizer uses vehicle coordinates and since simulator works in global map coordinates, reference trajectory points need to be transformed into vehicle coordinates. (main.cpp line 63 - uses helper.h `convert_space` function)

#### Latency compensation
Simulator has builtin latency=100 millis. To make sure that the optimizer works with most present state that latency needs to be compensated: Code lines 69 - 90 (with explanation).


#### Optimization time horizon
The model uses N (number of steps) = 10 and dt (time step) = 0.1 to calculate vehicle state.  
That means that models uses 1 second as a horizon to calculate optimal sequence of actuations.  
Cahnge cost for:
* N - increasing it allows to take longer future reference trajectory into account when computing the optimal values for present and require longer computational time.
* dt - decreasing improves approximation but decreases time frame.

I have tested it within N=[10,15] and dt=[0.05, 0.3]: 10 and 0.1 showed the best from prerformance:precision perspectives. 

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

