# Turtlebot Maze Solving - Obstacle Avoidance

This repository contains a Python implementation of a maze solving algorithm for turtlebot using obstacle avoidance. We use data obtained from the laser scan topic for avoiding obstacles and data from odom topic to calculate the displacement vector between initial and final position. The package can also be used as a good example for learning how nodes, topics, services and actions work in ROS environment.

# Dependencies for running the code

- ROS Version Kinetic (Full Installation)
- ROS Gazebo
- Python 2.7 Interpreter
- ROS Turtlebot (All related packages)

# How to run the project
```
cd ~/catkin_ws/src
git clone 
cd ..
catkin_make
source devel/setup.bash
roslaunch turtlebot_maze_solving main.launch
```
