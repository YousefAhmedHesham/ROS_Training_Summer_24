# TurtleBot3 Navigation 

## Overview

This project allows you to control a TurtleBot3 robot in a Gazebo simulation environment and navigate it to a specific goal.

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

1. Export your TurtleBot3 model:
    ```bash
    export TURTLEBOT3_MODEL=burger
    ```

2. Launch the TurtleBot3 in Gazebo:
    ```bash
    roslaunch turtlebot3_gazebo turtlebot3_world.launch
    ```

3. Start the navigation stack with your map:
    ```bash
    roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/path/to/your/map.yaml
    ```

4. Run the Python script to move the TurtleBot3 to the desired goal:
    ```bash
    python3 turtlebot3_navigation.py
    ```

## Usage

1. After launching the TurtleBot3 in Gazebo and starting the navigation stack, you can set a goal location by running the `turtlebot3_navigation.py` script.

2. The script will prompt you to input the x and y coordinates of the goal position.

3. The TurtleBot3 will navigate to the specified coordinates.

4. The terminal will output whether the goal was reached successfully or if there was an issue.
