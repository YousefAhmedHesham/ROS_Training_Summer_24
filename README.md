# TurtleBot3 Navigation with ROS

## Overview

This repository contains the implementation of a TurtleBot3 controller using ROS (Robot Operating System). The project allows the TurtleBot3 robot to navigate autonomously to specified coordinates on a map using ROS's `move_base` package.

## Features

- **Goal Navigation**: The robot can move to a specified goal position using the `move_base` action server.
- **User Input**: The x and y coordinates of the goal can be specified by the user through the command line.
- **Obstacle Avoidance**: The robot uses its sensors to avoid obstacles while navigating to the goal.

## Requirements

- **ROS Noetic**: This project is developed and tested on ROS Noetic.
- **TurtleBot3**: The project is designed for the TurtleBot3 robot.
- **Python 3**: The scripts are written in Python 3.
- **Gazebo**:
- **Dependencies**:
  - `move_base_msgs`
  - `actionlib`
  - `geometry_msgs`
  - `rospy`

## Setup

1. **Set the TurtleBot3 model**:
   ```bash
   export TURTLEBOT3_MODEL=burger
