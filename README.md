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

The total elapsed duration (N*dt) should be long enough. Short total duration causes overfit of the optimization and fails to find an appropriate solution of steering and throttle. For the same N*dt,  
larger N and smaller dt gives more accurate prediction than smaller N and larger dt, but more discretization also requires more computational resources. I found that N=10 and dt=0.01 work reasonably well on my computer. 

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

## Cost function

The terms in the cost function can be divided into three categories:

- Minimize the errors with the reference state.
- Minimize the use of actuators.
- Minimize the differences between the values of sequential throttle and steering angle.

I found that adding more weight on the difference between sequential steering values gives better results. I choose 200 in my implementation. The code snippet of the cost function is shown as below:


```cpp
// The part of the cost based on the reference state.
for (int t = 0; t < N; t++) {
    fg[0] += CppAD::pow(vars[cte_start + t], 2);
    fg[0] += CppAD::pow(vars[epsi_start + t], 2);
    fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
}

// Minimize the use of actuators.
for (int t = 0; t < N - 1; t++) {
    fg[0] += CppAD::pow(vars[delta_start + t], 2);
    fg[0] += CppAD::pow(vars[a_start + t], 2);
}

// Minimize the value gap between sequential actuations.
for (int t = 0; t < N - 2; t++) {
    fg[0] += 200 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
    fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}

```



## Model Predictive Control with Latency

Since a car needs to time to react to the change of its state, we have to take into account this delay in our model. My approach to solve this is adding another constraints into the optimization problem:
for the first K points such that K*dt<latency, the steering angles and and throttle is the same as their previous states.

For example, the following code snaps sets the range of delta based on this new constraint:
```c
for (int i = delta_start; i < a_start; i++) {

    // Take into account latency,
    // we bound the first few points to its previous value
    if ((i - delta_start) < latency_index) {
        vars_lowerbound[i] = prev_delta;
        vars_upperbound[i] = prev_delta;
    } else {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }

}
```

The code that sets the ranges of throttle is similar.



