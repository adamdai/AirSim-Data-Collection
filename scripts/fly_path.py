import airsim_data_collection.common.setup_path
import airsim

import numpy as np
import sys
import time

print("""Fly a path in an Unreal Engine environment""")

# ===================== Environment and path parameters ===================== #

PLAYER_START = np.array([-1647.25354, -2999.841553, 366.904236])

WAYPOINTS = np.array([[6360.0, -2930.0, 570.0],
             [6760.0, -520.0, 560.0],
             [6760.0, 3570.0, 560.0],
             [6280.0, 3900.0, 560.0],
             [6280.0, 7480.0, 640.0],
             [6750.0, 8030.0, 640.0],
             [6630.0, 11310.0, 640.0],
             [6000.0, 12000.0, 640.0],
             [-3000.0, 11880.0, 600.0],
             [-2920.0, 7410.0, 600.0],
             [-3040.0, -2780.0, 600.0]])

path_len = len(WAYPOINTS)

path = []
for i in range(path_len):
    waypt = (WAYPOINTS[i] - PLAYER_START) / 100.0
    path.append(airsim.Vector3r(waypt[0], waypt[1], -waypt[2]))




# MoveOnPath settings (https://microsoft.github.io/AirSim/apis/#apis-for-multirotor)
#drivetrain = airsim.DrivetrainType.MaxDegreeOfFreedom  
drivetrain = airsim.DrivetrainType.ForwardOnly
yaw_mode = airsim.YawMode(False, 0)  # (yaw_or_rate, is_rate)
                        
speed = 4    # speed
timeout = 300  # seconds
lookahead = 15  # how far to look ahead, default = -1 (auto-decide)
adaptive_lookahead = 1  # whether to apply adaptive lookahead, default = 1 (yes)



# ========================== Main Code ========================== #

# Initialize airsim client
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

# Arm and takeoff
print("arming the drone...")
client.armDisarm(True)

state = client.getMultirotorState()
if state.landed_state == airsim.LandedState.Landed:
    print("taking off...")
    client.takeoffAsync().join()
else:
    client.hoverAsync().join()

time.sleep(1)

state = client.getMultirotorState()
if state.landed_state == airsim.LandedState.Landed:
    print("take off failed...")
    sys.exit(1)


print("flying on path...")
client.startRecording()

try:
    result = client.moveOnPathAsync(path, speed, timeout,
                            drivetrain, yaw_mode, lookahead, adaptive_lookahead).join()

except KeyboardInterrupt:
    print("disarming...")
    client.armDisarm(False)
    print("resetting")
    client.reset()
    client.enableApiControl(False)
    print("done.")


# # drone will over-shoot so we bring it back to the start point before landing.
# client.moveToPositionAsync(0,0,z,1).join()

client.stopRecording()

print("landing...")
client.landAsync().join()

print("disarming...")
client.armDisarm(False)
print("resetting")
client.reset()
client.enableApiControl(False)
print("done.")
