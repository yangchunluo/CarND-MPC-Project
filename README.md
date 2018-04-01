## MPC Controller Project
---
Yangchun Luo<br>
April 1, 2018

This is the assignment for Udacity's Self-Driving Car Term 2 Project 5.

---
This project revisits the lake race track from the Behavioral Cloning Project. This time, however, we implement a MPC (model predictive control) controller in C++ to maneuver the vehicle around the track.

### To build

```bash
mkdir build
cd build
cmake ..
make
```

### To run

Download the [Term 2 simulator suite](https://github.com/udacity/self-driving-car-sim/releases). Open it and choose the Kidnapped Vehicle project.

Other setup information can be found in the original [README](README-orig.md) file.

To run the MPC controller:

```bash
./mpc
```

### Model Description

The model includes the following states and actuators. Note that all coordinates are converted to vehicle coordinate system at the initial state.

Vehicle states:

- `x`: vehicle's X coordinate
- `y`: vehicle's Y coordinate
- `v`: vehicle's velocity
- `psi`: vehicle's heading direction

Error terms (to be minimized):

- `cte`: cross-track error wrt. reference trajectory
- `epsi`: error in heading direction wrt. reference trajectory

Actuators (to be tuned/searched):

- `delta`: vehicle's steering angle
- `a`: vehicle's acceleration/brake

I employed a global kinematics model using the following state update rules:

- `x`[t+1] = `x`[t] + `v`[t] * cos(`psi`[t]) * `dt`
- `y`[t+1] = `y`[t] + `v`[t] * sin(`psi`[t]) * `dt`
- `psi`[t+1] = `psi`[t] + `v`[t] / `Lf` * `delta`[t] * `dt`
- `v`[t+1] = `v`[t] + `a`[t] * `dt`

For error terms, I always use (actual value - reference/desired value). I ended up discovering a mistake in the presentation of the course material in the update rule of `cte`[t+1]. I posted my discussion in [this forum topic](https://discussions.udacity.com/t/cte-update-wrt-epsi-might-be-a-mistake-in-course-material/651736).

- `cte`[t+1]  = `y`[t] - f(`x`[t]) - `v`[t] * sin(`epsi`[t]) * `dt`
- `epsi`[t+1] = (`psi`[t] - atan(f'(`x`[t])) + `v`[t] * `delta`[t] / `Lf` * `dt`
      
where `dt` is the time elapsed between t and t+1, `Lf` is the distance of vehicle's front wheel to its center of gravity, and f is the fitted polynomial of the reference trajectory.

### Choosing timestep length and elapsed duration

The parameter `N` (timesteps) and `dt` (each step's length) were chosen as 10 and 0.1s. So the total amount of time optimized by the MPC solver was 1 second. With a target driving speed of 100 MPH, this corresponds to 44 meters ahead. In typical driving, the road condition changes fast and it does not make too much sense to predict/optimize beyond a few seconds.

A smaller `dt` can smooth the state transition and reduce discretization error. However, given the same total length, reducing `dt` increases `N`, which is the major cost of computation done by the MPC solver. A slower response adds to the latency of actuating the vehicle. Or a capped computation time may not be enough to find the optimal path.

Given the above constraints and considerations, I believe the aforementioned values were reasonable. In simulation, it worked well for my hardware as well. 

### Fitting polynomial and preprocessing

The received waypoints (reference trajectory) were in map coordinate system. They were first translated to the vehicle's coordinate system using the following math equations.

- x = (my - vy) * sin(vpsi) + (mx - vx) * cos(vpsi)
- y = (my - vy) * cos(vpsi) - (mx - vx) * sin(vpsi)

where mx, my are a waypoint's map coordinate, vx, vy, vpsi are the vehicle's location and heading in map coordinate system as well.

After such translation, all points are centered at the vehicle (origin) with X axis being its heading direction. This simplifies the polynomial fitting process in that the waypoints (road ahead) can be represented by a single variable polynomial function that is aligned with the X axis (i.e. not a rotated polynomial which requires a parametric representation).

Then a 3rd degree polynomial is fit to the translated waypoints. The effectiveness is verified by sending the translated points back to the simulation and the displayed line closely matches with the center of the road.

### Dealing with actuator latency

In the client code I simulated a 100ms actuator latency (i.e. the time between actuator inputs are calculated and such control is actually achieved).

MPC offers an easy to factor in this latency. Given the current vehicle states, I calculated the new state past the duration of the latency using the same kinematics model. The new state is used as the initial state fed into the MPC solver.

There can be many way to derive the new state. I chose the following equations based on the assumption that latency is very short:

- x = v * latency
- y = v * latency * delta * latency
- psi = v * delta / Lf * latency

For error terms, use (actual - reference) value

- cte = y - f(x)
- epsi = psi - atan(f'(x))

Update velocity last

- v += a * latency

### Result

The result of MPC controlled car running one lap of the lake track can be found [here](result.mp4). When the reference speed was set to 100 MPH, the car optimistically slowed down at sharp turns and accelerated passing the turn. I clocked one lap's time was around 36.5 seconds.
