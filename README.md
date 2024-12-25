# Basketball Hoop Control System - IEEE NIT-K Robotrix 
# Project Overview
 This project is developed for the IEEE Robotrix 2024-25 hackathon. The goal is to design a basketball hoop control system that dynamically tracks and catches basketballs using vision sensors, robotic controls, and mechanical rails in a simulated environment. The system is implemented in CoppeliaSim using the ZeroMQ Remote API.

# Features
 Real-time Ball Detection: Utilizes HSV filtering and contour detection to identify the basketball's position in the vision sensor's frame.

 
Dynamic Hoop Movement:


Front/Back Movement: Managed by rail_1.


Left/Right Movement: Managed by rail_2.


Vertical Movement: The hoop itself adjusts vertically.


Reset Mechanism: Automatically resets the hoop and rails to their initial positions after catching the ball or when the ball is not detected.


Keyboard Controls:


q to quit the simulation.


# Requirements
# Software
CoppeliaSim: Simulation environment. Download it from https://www.coppeliarobotics.com/.

# Setup Instructions
# Step 1: Configure CoppeliaSim
Open CoppeliaSim and load a simulation scene that includes:


Two vision sensors (/cam_r and /cam_l).


A basketball hoop (/hoop).


Two rails for movement (/rail_1 and /rail_2).


Ensure that the simulation scene is calibrated for ball tracking.


# Step 2: Run the Python Script
Open a terminal and navigate to the project directory.

Run the script
# License
This project is intended for IEEE Robotrix 2024-25. Redistribution or modification for purposes outside the competition must comply with its rules and guidelines.
