'''
*****************************************************************************************
*
*        =================================================
*                    IEEE Robotrix 2024-25
*        =================================================
*
*  This script is intended for implementation of the final hackathon
*  task of IEEE Robotrix 2024-25
*
*****************************************************************************************
'''

# Team Name:
# Team Members:
# Filename:         task.py
# Functions:        control_logic
#                   [ Comma-separated list of functions in this file ]
# Global variables:
#                   [ List of global variables defined in this file ]

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
##############################################################

################# ADD GLOBAL VARIABLES HERE ##################
lower_blue = np.array([100, 50, 100])
upper_blue = np.array([140, 255, 255])
##############################################################

################# ADD UTILITY FUNCTIONS HERE #################

def detect_ball(sim, vision_sensor_handles):
    """
    Detects the ball using the vision sensors and returns its coordinates and image resolution.
    """
    best = None
    for i, sensor in enumerate(vision_sensor_handles):
        try:
            img_buf, res = sim.getVisionSensorImg(sensor)
            if not img_buf:
                continue
            img = np.frombuffer(img_buf, dtype=np.uint8).reshape((res[1], res[0], 3)).copy()
            img = cv2.flip(img, 0)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            cont, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cont:
                cont = sorted(cont, key=cv2.contourArea, reverse=True)
                area = cv2.contourArea(cont[0])
                if area > 100:  # Avoid noise
                    x, y, w, h = cv2.boundingRect(cont[0])
                    cx, cy = x + w // 2, y + h // 2
                    best = (cx, cy, res)
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 128, 0), 2)
                    cv2.circle(img, (cx, cy), 5, (0, 255, 255), -1)
            cv2.imshow(f"Vision {i+1}", img)
        except Exception as e:
            print(f"[ERROR] Vision sensor {i+1} error: {e}")
    return best

def track_ball(sim, hoop, rail1, rail2, info, off1, off2, step=0.1, th=10):
    """
    Tracks the ball and moves the hoop to align with its position.
    """
    if not info:
        return
    cx, cy, (w, h) = info
    ph = sim.getObjectPosition(hoop, -1)
    cx_c, cy_c = w // 2, h // 2
    dx = dy = 0
    if cx < cx_c - th:
        dx = -step
    elif cx > cx_c + th:
        dx = step
    if cy < cy_c - th:
        dy = step
    elif cy > cy_c + th:
        dy = -step
    new_hoop = [ph[0] + dx, ph[1] + dy, ph[2]]
    sim.setObjectPosition(hoop, -1, new_hoop)
    r1 = [new_hoop[i] + off1[i] for i in range(3)]
    r2 = [new_hoop[i] + off2[i] for i in range(3)]
    sim.setObjectPosition(rail1, -1, r1)
    sim.setObjectPosition(rail2, -1, r2)

def reset_positions(sim, hoop, rail1, rail2, hopPos, hopOri, r1Pos, r1Ori, r2Pos, r2Ori):
    """
    Resets all objects to their initial design-time positions and orientations.
    """
    sim.setObjectPosition(hoop, -1, hopPos)
    sim.setObjectOrientation(hoop, -1, hopOri)
    sim.resetDynamicObject(hoop)

    sim.setObjectPosition(rail1, -1, r1Pos)
    sim.setObjectOrientation(rail1, -1, r1Ori)
    sim.resetDynamicObject(rail1)

    sim.setObjectPosition(rail2, -1, r2Pos)
    sim.setObjectOrientation(rail2, -1, r2Ori)
    sim.resetDynamicObject(rail2)

    print("[DEBUG] Reset all to initial state.")

##############################################################

def control_logic(sim):
    """
    Implements the control logic to track and catch the basketball.
    """
    cr = sim.getObject('/cam_r')
    cl = sim.getObject('/cam_l')
    vision_sensors = [cr, cl]

    hoop = sim.getObject('/hoop')
    rail1 = sim.getObject('/rail_1')
    rail2 = sim.getObject('/rail_2')

    hopPos = sim.getObjectPosition(hoop, -1)
    hopOri = sim.getObjectOrientation(hoop, -1)
    r1Pos = sim.getObjectPosition(rail1, -1)
    r1Ori = sim.getObjectOrientation(rail1, -1)
    r2Pos = sim.getObjectPosition(rail2, -1)
    r2Ori = sim.getObjectOrientation(rail2, -1)

    off1 = [r1Pos[i] - hopPos[i] for i in range(3)]
    off2 = [r2Pos[i] - hopPos[i] for i in range(3)]

    print("Press 'q' to quit, 'r' to reset.")

    last_detected_time = time.time()

    try:
        while True:
            info = detect_ball(sim, vision_sensors)
            if info:
                last_detected_time = time.time()
                track_ball(sim, hoop, rail1, rail2, info, off1, off2, step=0.1, th=10)
            else:
                if time.time() - last_detected_time > 2:
                    print("[DEBUG] Ball not detected for 2 seconds. Resetting positions.")
                    reset_positions(sim, hoop, rail1, rail2, hopPos, hopOri, r1Pos, r1Ori, r2Pos, r2Ori)
                    last_detected_time = time.time()
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            elif k == ord('r'):
                reset_positions(sim, hoop, rail1, rail2, hopPos, hopOri, r1Pos, r1Ori, r2Pos, r2Ori)
            time.sleep(0.01)
    except Exception as e:
        print("[ERROR]", e)
    finally:
        reset_positions(sim, hoop, rail1, rail2, hopPos, hopOri, r1Pos, r1Ori, r2Pos, r2Ori)
        time.sleep(0.1)

######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THE MAIN CODE BELOW #########

if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.getObject('sim')    

    try:
        sim.startSimulation()
        control_logic(sim)
        sim.stopSimulation()
    except Exception as e:
        print("[ERROR]", e)
    finally:
        sim.stopSimulation()
        cv2.destroyAllWindows()
