'''
*****************************************************************************************
*
*        =================================================
*                    IEEE Robotrix 2024-25
*        =================================================
*
*  This script is intended for implementation of the final hackaton
*  task of IEEE Robotrix 2024-25
*
*****************************************************************************************
'''

# Team Name:
# Team Members:
# Filename:			task.py
# Functions:		control_logic
# 					[ Comma separated list of functions in this file ]
# Global variables:
# 					[ List of global variables defined in this file ]

####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
##############################################################
import sys
import traceback
import time
import os
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2
import random
##############################################################

################# ADD GLOBAL VARIABLES HERE ##################
# Global variables can be defined here if needed
##############################################################

################# ADD UTILITY FUNCTIONS HERE #################
# Utility functions can be added if needed
##############################################################

def control_logic(sim):
    """
    Purpose:
    ---
    This function should implement the control logic for the given problem statement
    You are required to make the robot move the hoop to catch the basketball.

    Input Arguments:
    ---
    `sim`    :   [ object ]
        ZeroMQ RemoteAPI object

    Returns:
    ---
    None

    Example call:
    ---
    control_logic(sim)
    """
    
    # Get initial positions (you will replace this with actual calls later)
    hoop_position = sim.getObjectPosition('hoop_odom', -1)  # Example: get the position of the hoop
    
    # Initialize cameras or vision sensors (assuming cameras are identified as 'camera_left' and 'camera_right')
    camera_left = sim.getObject('camera_left')
    camera_right = sim.getObject('camera_right')
    
    # Step 1: Process images from the vision sensors to detect the basketball
    # Assuming function 'detect_ball_from_camera' exists or needs to be implemented
    ball_position = detect_ball_from_camera(camera_left, camera_right)  # Detect ball position
    
    # Step 2: If ball detected, calculate its trajectory (you may need to use Kalman filter or another method)
    if ball_position is not None:
        # Calculate predicted landing position (or simply move the hoop to the ball's current position)
        predicted_position = predict_ball_trajectory(ball_position)
        
        # Step 3: Move the hoop to intercept the ball's path
        move_hoop_to_position(sim, predicted_position)
    
    else:
        # If the ball is not detected, you could either wait or attempt to reposition the hoop
        # Step 4: Optionally, implement a strategy for searching for the ball if it is not detected
        pass
    
    # Optional: Add a condition to stop when the ball is dunked (when it's within hoop)
    if is_ball_in_hoop(hoop_position, ball_position):
        print("Ball dunked successfully!")
        # Stop or reset simulation, if needed
    
    return None

def detect_ball_from_camera(camera_left, camera_right):
    """
    Function to detect the ball using the cameras.
    This is a placeholder function; implement the actual detection logic.
    """
    # Example logic: Process images from cameras
    # You can use OpenCV or other image processing techniques to find the ball
    # For now, returning a random position to simulate detection
    ball_x = random.randint(0, 512)  # Random X position of ball in camera image
    ball_y = random.randint(0, 512)  # Random Y position of ball in camera image
    return np.array([ball_x, ball_y])  # Return the ball position as an array

def predict_ball_trajectory(ball_position):
    """
    Function to predict the trajectory of the basketball.
    Placeholder logic to determine where the ball is going based on its current position and velocity.
    """
    # Implement ball trajectory prediction here (e.g., using a simple physics model or Kalman filter)
    # For now, assume the ball is going straight to the hoop's position
    predicted_position = ball_position + np.array([0, 100])  # Simulate a basic trajectory
    return predicted_position  # Return predicted position

def move_hoop_to_position(sim, position):
    """
    Function to move the hoop to the desired position.
    """
    # Assuming the hoop's velocity can be controlled directly:
    # Example code to move hoop (replace with actual control method)
    sim.setObjectVelocity('hoop_odom', [0, 0, 0])  # Stop the hoop (if moving)
    sim.setObjectPosition('hoop_odom', position)  # Move hoop to new position

def is_ball_in_hoop(hoop_position, ball_position):
    """
    Function to check if the ball is inside the hoop.
    """
    # Simple check: if the ball is within the hoop's position range
    hoop_radius = 320 / 2  # Half of hoop diameter
    distance = np.linalg.norm(np.array(hoop_position) - np.array(ball_position))
    return distance < hoop_radius


######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THE MAIN CODE BELOW #########

if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.getObject('sim')    

    try:

        ## Start the simulation using ZeroMQ RemoteAPI
        try:
            return_code = sim.startSimulation()
            if sim.getSimulationState() != sim.simulation_stopped:
                print('\nSimulation started correctly in CoppeliaSim.')
            else:
                print('\nSimulation could not be started correctly in CoppeliaSim.')
                sys.exit()

        except Exception:
            print('\n[ERROR] Simulation could not be started !!')
            traceback.print_exc(file=sys.stdout)
            sys.exit()

        ## Runs the control logic written by participants
        try:
            control_logic(sim)

        except Exception:
            print('\n[ERROR] Your control_logic function threw an Exception, kindly debug your code!')
            print('Stop the CoppeliaSim simulation manually if required.\n')
            traceback.print_exc(file=sys.stdout)
            print()
            sys.exit()

        
        ## Stop the simulation
        try:
            return_code = sim.stopSimulation()
            time.sleep(0.5)
            if sim.getSimulationState() == sim.simulation_stopped:
                print('\nSimulation stopped correctly in CoppeliaSim.')
            else:
                print('\nSimulation could not be stopped correctly in CoppeliaSim.')
                sys.exit()

        except Exception:
            print('\n[ERROR] Simulation could not be stopped !!')
            traceback.print_exc(file=sys.stdout)
            sys.exit()

    except KeyboardInterrupt:
        ## Stop the simulation
        return_code = sim.stopSimulation()
        time.sleep(0.5)
        if sim.getSimulationState() == sim.simulation_stopped:
            print('\nSimulation interrupted by user in CoppeliaSim.')
        else:
            print('\nSimulation could not be interrupted. Stop the simulation manually .')
            sys.exit()
