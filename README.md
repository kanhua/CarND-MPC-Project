# CarND-Controls-MPC
This project is part of Self-Driving Car Engineer Nanodegree Program.
Details on how to run this project is [here](./install.md).

---

## The model

The equation of motion of the vehicle is described as below.

![eqns](./eqns.png)

where x_t, y_t, psi_t are the coordinates and orientation of the car with respective to the car at time t,
delta is the steering angle, a_t is the throttle, and L_f measures the distance between the front of the vehicle and its center of gravity. 


## Timestep Length and Elapsed Duration (N & dt)




## Polynomial Fitting and MPC Preprocessing

The waypoints are fitted by the third-order polynomials:

```c
for (int i = 0; i < ptsx.size(); i++) {
    double dx = ptsx[i] - px;
    double dy = ptsy[i] - py;

    next_x_vals.push_back(dx * cos(psi) + dy * sin(psi));
    next_y_vals.push_back(dx * sin(-psi) + dy * cos(psi));
    way_x[i] = next_x_vals[i];
    way_y[i] = next_y_vals[i];
}
```


## Model Predictive Control with Latency

Since a car needs to time to react to the change of its state, we have to take into account this delay in our model. My approach to solve this is adding another constraints into the optimization problem:
for the first K points such that K*dt<latency, the steering angles and and throttle is the same as their previous states.

For example, the following code snaps sets the range of delta based on this new constraint:
```c
for (int i = delta_start; i < a_start; i++) {

    // Take into account latency
    if (i < latency_index) {
        vars_lowerbound[i] = prev_delta;
        vars_upperbound[i] = prev_delta;
    } else {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }

}
```

The code that sets the ranges of throttle is similar.



