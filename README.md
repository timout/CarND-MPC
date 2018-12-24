# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

[formulae]: state_update.png

---

## Objective
This project is to use Model Predictive Control (MPC) to drive a car in a game simulator. The server provides 
reference waypoints (yellow line in the demo video) via websocket, and we use MPC to compute steering and throttle commands 
to drive the car. The solution must be robust to 100ms latency, since it might encounter in real-world application.

In this project, the MPC optimize the actuators (steering and throttle), simulate 
the vehicle trajectory, and minimize the cost like cross-track error.

## Model

This project follows the Kinematic Model. Kinematic models are 
simplifications of dynamic models that ignore tire forces, gravity, and mass. This simplification reduces the accuracy 
of the models, but it also makes them more tractable.

It may be useful to distinguish three significant parts within it: *State*, *Actuators*, *Equations*.

**States**: 

* x: cars x position
* y: cars y position
* ψ (psi): vehicle's angle in radians from the x-direction (radians)
* ν: vehicle's velocity
* cte: cross track error
* eψ : orientation error

**Actuator values**:

* δ (delta): steering angle
* a : acceleration (including throttle and break)

**Update equations**:

![alt][formulae]

## Timestep Length and Elapsed Duration (N and dt)

* N = 10
* dt = 0.1 s 

The prediction horizon is the duration over which future predictions are made. We’ll refer to this as T.
T is the product of two other variables, T =  N * dt. In the case of driving a car, T should be a few seconds, 
at most. Beyond that horizon, the environment will change enough that it won't make sense to predict any further 
into the future.  N and dt are hyperparameters you will need to tune for each model predictive controller you build. 
However, there are some general guidelines: T should be as large as possible, while dt should be as small as possible.
These guidelines create tradeoffs.

The goal of Model Predictive Control is to optimize the control inputs: [δ,a]. An optimizer will tune these inputs 
until a low cost vector of control inputs is found. 

## Model Predictive Control with Latency

In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates 
through the system. A realistic delay might be on the order of 100 milliseconds, so in this project 100 millisecond 
latency is handled by Model Predictive Controller. 

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

