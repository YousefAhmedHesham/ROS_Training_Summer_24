# TurtleSim Goal Navigation

This Python script controls a turtle in the ROS (Robot Operating System) `turtlesim` simulator. The turtle moves towards a specified goal position within the simulator.

## Overview

The script utilizes ROS to control the turtle's movements by publishing velocity commands and subscribing to the turtle's pose. It uses proportional control to adjust both linear and angular velocities to guide the turtle to the desired goal position.

## Features

- **Initialization**: Initializes ROS node, publishers, and subscribers.
- **Pose Callback**: Updates the turtle's current pose.
- **Move to Goal**: Computes and sends velocity commands to move the turtle to the specified goal coordinates.

## Usage

1. Ensure you have ROS installed and set up properly.
2. Run the turtlesim node:rosrun turtlesim turtlesim_node
3. rosrun <your_package_name> <your_script_name>.py
4. Input the desired x and y coordinates (both must be ≤ 11).
