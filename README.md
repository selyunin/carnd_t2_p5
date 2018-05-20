# Model Predictive Controller
Self-Driving Car Engineer Nanodegree Program


The goal of the project is to implement a model-predictive controller (MPC)
for calculating the steering control inputs for the vehicle given race track waypoints.

This project should be run in the Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

## Compiling and building

The project has been tested / run on Ubuntu.

* cmake >= 3.5
* make >= 4.1 (Linux, Mac),
* gcc/g++ >= 5.4
* Ipopt 
* CppAD
* uWebSockets

Cmake is used as a build system for the project. 
In order to compile the code run the commands below in the `bash` shell in the root project directory:
1. `mkdir build`
2. `cd build`
3. `cmake ../src/`
4. `make`
5. `./mpc`

Create Eclipse project files:
1. `cd build`
2. `cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../src/`

## Implementation

The disadvantage of the PID controller is that it acts only based on the past history (errors).
One can think of this as driving the car by only looking in the rear-view mirror, and not looking
in front of the car. Model predictive control addresses this drawback and uses the kinematic model 
of the car to predict the  behavior for a finite future horizon.
In essence, MPC formulates a control problem as an optimization problem, 
and this project showcases this approach.

### The Model

The kinematic model of the car has 
`6` states: `[x, y, psi, v, cte, epsi]` and 
`2` control inputs `[delta, a]`.

The update equations of the model are as follows:

`x[t+1] = x[t] + v[t] * cos(psi[t]) * dt`

`y[t+1] = y[t] + v[t] * sin(psi[t]) * dt`

`psi[t+1] = psi[t] - v[t] / Lf * delta[t] * dt`

`v[t+1] = v[t] + a[t] * dt`

`cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt`

`epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt`


Where: 

* `x` - x position of the vehicle,

* `y` - y position of the vehicle,

* `psi` - orientation of the vehicle,

* `v` - velocity of the vehicle,

* `cte` - cross track error,

* `epsi` - orientation error,

* `Lf` is the distance between the center of mass 
of the vehicle and it's front axle,

* `f(x[t])` is a third degree polynomial,

* `psides[t]` is equivalent to `arctan(f'(x[t]))`.

* `delta` - steering angle (control input),

* `a` - throttle or acceleration (control input). 

### Timestep Length and Elapsed Duration (N and dt)

The MPC computes car trajectory for a finite horizon N.
Since only the first control input will be taken, it is of no need
to compute extremely large horizons (they will be re-computed in the 
next time stamp anyway). 
Time step dt is used to compute discrete approximation of the 
kinematic model of the vehicle large dt values will account for 
big error. 

I tried `dt` of `0,025, 0.05, 0.1, 0.2` and `N` of `10, 20, 25`. 
Small dt did not really worked for me (less then `0.1`, may be 
due to numerical issues with the optimization). `0.1` and `0.2` works
fine, although I find dt of `0.1` is a bit more stable.

### Polynomial Fitting and MPC Preprocessing

MPC controller receives from a simulator a JSON object with the 
waypoints in global coordinates. 
These waypoints are transformed to the local car
coordinates and then the 3rd degree  polynomial 
is fitted to these waypoints.

### Model Predictive Control with Latency

We handle latency in the following way. 
We first take current speed and acceleration of the car from the simulator.
We then assume that the car continues to drive (in its local coordinates)
for a *latency* amount of time with this speed and acceleration. 
This gives us an updated state, which is used as input for the MPC.

MPC itself takes variables, constraints and the cost function and uses
interior point optimization to find variables that satisfy the constraints 
and give the lowest cost.
Let's break it up:

* Cost function (`fg[0]`) is a weighted some of quantities that should be 
minimized: 
(i) cross-track error, 
(ii) orientation error, 
(iii) difference between the current and the target speed,
(iv) use of actuators,
(v) gap between sequential actuations.
Each quantity (i) - (v) is penalized in the cost function with a weight: 
the larger weight depicts important quantity during optimization.

* Constraints puts restriction on the variables:
(i) start state cannot be changed,
(ii) the following states should follow the kinematic model,
(iii) actuator values are bounded.

* Variables: state vector for each time stamp and the actuator values
for time stamps `[1, N]` (we do not change state for t stamp `0`).

The optimization then return values of the variables with the minimal 
cost function. We apply first control inputs, send it to the simulator,
and upon receiving the next state from the simulator recalculate.
This allows use simple kinematic model of a car to find plausible control
input via optimization.

