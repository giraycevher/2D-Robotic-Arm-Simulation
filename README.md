# 2-DOF Planar Robot Manipulator Control Simulation
%Giray Cevher Eser

I would like to thank Raşit Evdüzen for his supervision and significant contributions to the development of this code.


This project implements a complete **Dynamic Modeling and Control** framework for a 2-DOF planar robotic arm. It demonstrates the application of **Lagrangian Dynamics**, **Computed Torque Control (CTC)**, and **S-Curve Trajectory Planning** to achieve high-precision motion with minimized jerk.

## Key Features

Advanced Trajectory Planning:Uses Quintic Polynomials and Double S-Curve profiles to ensure continuous acceleration and reduced mechanical stress (Jerk control).
  Physics-Based Modeling:** Full dynamic model derivation using the Lagrangian Formulation, including explicit calculation of Mass, Centripetal/Coriolis, and Gravity matrices.
  Nonlinear Control:** Implementation of **Computed Torque Control (CTC)** (Feedback Linearization) for precise trajectory tracking.
  Inverse Kinematics:** Geometric solution for 2-DOF planar manipulator.

##  Tech Stack

Language: MATLAB
Core Concepts: Robotics, Control Theory, System Dynamics, Numerical Simulation

#Simulation Results

<img width="1641" height="754" alt="image" src="https://github.com/user-attachments/assets/e3b54348-0258-4e52-a134-57baf57db4ed" />




> The graph above demonstrates the computed torque values required for the manipulator to follow the planned trajectory under gravity and dynamic effects.

## Project Structure

* `main.m`: Entry point for the simulation. Runs the control loop and visualizations.
* `Mass_Matrix.m`: Calculates the inertia matrix $M(q)$.
* `Centripetal_Matrix.m`: Calculates Coriolis and Centrifugal terms $C(q, \dot{q})$.
* `Gravity_Matrix.m`: Computes gravity vectors $G(q)$.
* `fifth_order_trajectory.m`: Generates jerk-limited trajectory profiles.

# How to Run

1.  Clone the repository:
    ```bash
    git clone [https://github.com/giraycevher/2dof-robot-sim.git](https://github.com/giraycevher/2dof-robot-sim.git)
    ```
2.  Open `main.m` in MATLAB.
3.  Run the script. The simulation window will visualize the robot motion and torque profiles.


---
*Developed by Giray Cevher Eser*
