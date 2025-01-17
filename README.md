# SmartCleaningRobot

A growing robot vacuum cleaner project, the goal is to build an open source robot vacuum cleaner project for educational learning.

## The Origin

I have developed an initial cleaning robot project at Brandeis University Robotics Labs: 
* https://github.com/campusrover/cleaning_robot

According to the future plan of this project, I am now officially continuing to delve into the project.

This original cleaning robot project is based on ROS Noetic and Ubuntu20.04, and use some ROS packages to achieve exploration functions. I designed the cleaning functions, and use AMCL to localize. Now I am modifying some parts, and transfering them into ROS2, and would like to redesign the whole system. The new system in my opinion will have multi-agent collaborations, RL expansion space, etc.

## New SmartCleaningRobot Proposal

Based on previous work, in this new project, I will add following feature:
* Intelligent Agent built in edge device, use Jetson Nano to increase the power.
* DIY exploration and mapping module
* Update cleaning module
* New control panel based on phone
* Connect with other IoT devices
