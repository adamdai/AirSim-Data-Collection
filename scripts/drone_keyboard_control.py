"""Drone keyboard control

"""

import airsim
import keyboard
import numpy as np


# Parameters
# ------------------------------
speed = 5.0     # m/s
dt = 0.1        # s

# Main
# ------------------------------
if __name__ == "__main__":

    # Connect to client
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("Arming drone...")
    client.enableApiControl(True)
    client.armDisarm(True)

    print("Press space to take off")
    keyboard.wait('space')
    client.takeoffAsync().join()

    print("Ready for control. Press Ctrl+C to exit.")
    try:
        while True:
            v = np.zeros(3)  # [vx, vy, vz]
            yaw = 0.0        # yaw angle in radians
            rpyt = np.zeros(4)  # [roll, pitch, yaw, thrust]

            if keyboard.is_pressed('up'):
                v[0] = speed
                rpyt[1] = 0.2
            if keyboard.is_pressed('left'):
                v[1] = -speed
                rpyt[0] = -0.2
            if keyboard.is_pressed('down'):
                v[0] = -speed
                rpyt[1] = -0.2
            if keyboard.is_pressed('right'):
                v[1] = speed
                rpyt[0] = 0.2
            if keyboard.is_pressed('w'):
                v[2] = -speed
                rpyt[3] = 1.0
            if keyboard.is_pressed('s'):
                v[2] = speed    
            if keyboard.is_pressed('a'):
                yaw = 0.5
                rpyt[2] = 0.5
            if keyboard.is_pressed('d'):
                yaw = -0.5
                rpyt[2] = -0.5
            
            client.moveByVelocityAsync(v[0], v[1], v[2], dt).join()
            #client.rotateByYawRateAsync(yaw, dt).join()
            #client.moveByRollPitchYawrateThrottleAsync(rpyt[0], rpyt[1], rpyt[2], rpyt[3], dt).join()



    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Exiting...")
        client.reset()
        client.armDisarm(False)
        client.enableApiControl(False)
        exit()



        

    



