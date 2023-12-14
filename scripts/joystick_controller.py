#!/usr/bin/env python

"""
Joystick controller for the drone
(tested with xbox 360 wireless controller)

"""

import redis
import time
import airsim
import numpy as np
import pygame
import os
from scipy.spatial.transform import Rotation as R


if __name__ == '__main__':


    pygame.init()
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print("Connectd to joystick: ", joy.get_name())

    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    client.takeoffAsync().join()

    # Hover to start position
    client.moveToPositionAsync(-6.84, 0, -9.68, 5).join()

    # Axes: 
    # 0: left stick left(-1)/right(+1)     - yaw
    # 1: left stick up(-1)/down(+1)        - z
    # 2: right stick left(-1)/right(+1)    - y
    # 3: right stick up(-1)/down(+1)       - x (negative)

    dt = 0.001
    speed = 5
    deadzone = 0.1

    while(True):
        try:
            pygame.event.pump()

            # curr_rot = client.simGetVehiclePose().orientation
            # curr_rot = np.array([curr_rot.x_val, curr_rot.y_val, curr_rot.z_val, curr_rot.w_val])
            # scipy_rot = R.from_quat(curr_rot)
            # yaw = scipy_rot.as_euler('xyz')[2]
            # print("yaw: ", yaw)

            yaw_rate = 90 * joy.get_axis(0)
            vx = speed * -joy.get_axis(3)
            vy = speed * joy.get_axis(2)
            vz = speed * joy.get_axis(1)

            #print("vx: ", vx, "vy: ", vy, "vz: ", vz, "yaw_rate: ", yaw_rate)
            
            # TODO: add deadzone stabilization
            if abs(vz) < deadzone:
                vzHold = client.getMultirotorState().kinematics_estimated.position.z_val
                client.moveByVelocityZBodyFrameAsync(vx, vy, vzHold, 
                                                    duration=dt,
                                                    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate)).join()
            else:
                client.moveByVelocityBodyFrameAsync(vx, vy, vz, 
                                                    duration=dt,
                                                    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate)).join()
        
        except KeyboardInterrupt:
            print("Keyboard Interrupt!")
            break


    client.armDisarm(False)
    client.enableApiControl(False)
    client.reset()
    