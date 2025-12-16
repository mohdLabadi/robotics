# Robotics (CS 4750) Homeworks and Final Project) (ROS)

This repository contains a series of robotics programming assignments implemented using ROS, NumPy, and Python. The projects progress from core ROS concepts to motion modeling, state estimation, planning, and control, using both a mobile robot platform (MuSHR) and a 6 DOF manipulator (WidowX 250). Together, these assignments form a cohesive introduction to modern robotics software pipelines, combining mathematical foundations with practical system implementation.

## Contents

- [Homework 1: ROS Fundamentals and Simulation](#homework-1-ros-fundamentals-and-simulation)
- [Homework 2: Kinematics](#homework-2-kinematics)
- [Homework 3: Particle Filtering](#homework-3-particle-filtering)
- [Homework 4: Planning](#homework-4-planning)
- [Homework 5: Control](#homework-5-control)
- [Repository Structure](#repository-structure)
- [Setup and Running](#setup-and-running)
- [Notes](#notes)

## Homework 1: ROS Fundamentals and Simulation

This homework introduces the course ROS workflow by setting up the environment, practicing terminal basics, and learning how nodes communicate through topics with publishers and subscribers. It includes building a ROS publisher node that checks whether a parameter is a prime number and launching it through a custom launch file. It also walks through running and visualizing the MuSHR car simulator using RViz while inspecting nodes and topics with ROS command line tools. Finally, it covers NumPy vectorization and ROS data collection by subscribing to the car pose topic, computing Manhattan norms efficiently, and producing plots to compare runtime and visualize trajectories.

## Homework 2: Kinematics

This homework covers kinematic modeling and simulation for both a car like robot and a 6 DOF robotic arm in ROS. It introduces the Ackermann car model for the MuSHR platform, including deterministic and stochastic motion models, vectorized state propagation, and visualization of rollouts under noise. The assignment also explores arm kinematics using the WidowX 250, focusing on forward kinematics, coordinate frame transformations, Jacobians, and velocity based control. Together, these components provide a foundation for understanding motion modeling and kinematic control in mobile and manipulator robots.

## Homework 3: Particle Filtering

This homework implements particle filtering for both the MuSHR car and the WidowX 250 arm, with an emphasis on robust tracking under noise and partial observations. The car portion focuses on state estimation using a particle filter framework. The arm portion uses a toy visual servoing setup where an overhead camera observes a moving cylinder, and the task is to track the target over time despite noise and occlusions by implementing the particle filter algorithm.

## Homework 4: Planning

This homework implements A* and RRT planning algorithms for both the MuSHR car and the WidowX 250 arm. The main focus is on generalizing planning ideas across different platforms by defining state representations, validity checks, and search or sampling based expansions. These implementations build intuition for how discrete search and sampling based planning can be applied to navigation and manipulation.

## Homework 5: Control

This homework implements fundamental path tracking controllers, including PID and MPC, for accurately following desired trajectories. The assignment emphasizes parameter tuning and evaluating controller behavior under different settings, helping build intuition for stability, responsiveness, and tradeoffs between feedback control strategies.

## Repository Structure

Typical layout follows a ROS workspace style organization, with packages located under `src`.

```text
repo_root/
  src/
    hw1_introduction/
    hw2_kinematics/
    hw3_particle_filter/
    hw4_planning/
    hw5_control/
  README.md
  .gitignore
