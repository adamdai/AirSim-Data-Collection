import airsim_data_collection.common.setup_path
import airsim

import sys
import time

print("""Fly a path in an Unreal Engine environment""")

# ===================== Environment and path parameters ===================== #

# Starting offset (coordinates of PlayerStart object in meters)
x_init = -67.300
y_init = 15.600
z_init = 0.20

# AirSim initialization offset (initial X,Y,Z in AirSim/settings.json)
x_offset = 0.0
y_offset = 0.0
z_offset = -2.0

# Flying speed
speed = 1.0

# ========================== Functions ========================== #

# Transform from world frame (drone start at (0,0,0)) to AirSim frame
# Takes an (x,y,z) tuple as input and outputs an (x,y,z) tuple
def world2airsim(pos):
    return (pos[0] - x_offset,
            pos[1] - y_offset,
            pos[2] - z_offset)

# Transform from Unreal coordinates to AirSim coordinates
# Takes an (x,y,z) tuple as input and outputs an (x,y,z) tuple
def unreal2airsim(pos):
    wpos = (pos[0] - x_init,
            pos[1] - y_init,
            -(pos[2] - z_init))
    return world2airsim(wpos)


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

# Move to position
# upos = (-13.19, 15.6, 1.75)
# apos = unreal2airsim(upos)

# client.moveToPositionAsync(apos[0], apos[1], apos[2], speed).join()

# upos = (-13.19, -0.20, 1.75)
# apos = unreal2airsim(upos)

# client.moveToPositionAsync(apos[0], apos[1], apos[2], speed).join()

# upos = (-1.0, -0.20, 1.75)
# apos = unreal2airsim(upos)

# client.moveToPositionAsync(apos[0], apos[1], apos[2], speed).join()

print("flying on path...")

upos1 = (-13.19, 15.6, 1.75)
apos1 = unreal2airsim(upos1)
upos2 = (-13.19, -0.20, 1.75)
apos2 = unreal2airsim(upos2)
upos3 = (-1.0, -0.20, 1.75)
apos3 = unreal2airsim(upos3)

try:

    result = client.moveOnPathAsync([airsim.Vector3r(apos1[0],apos1[1],apos1[2]),
                                    airsim.Vector3r(apos2[0],apos2[1],apos2[2]),
                                    airsim.Vector3r(apos3[0],apos3[1],apos3[2])],
                            speed, 300,
                            airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), -1, 1).join()

except KeyboardInterrupt:
    print("disarming...")
    client.armDisarm(False)
    print("resetting")
    client.reset()
    client.enableApiControl(False)
    print("done.")


# # drone will over-shoot so we bring it back to the start point before landing.
# client.moveToPositionAsync(0,0,z,1).join()

print("landing...")
client.landAsync().join()

print("disarming...")
client.armDisarm(False)
print("resetting")
client.reset()
client.enableApiControl(False)
print("done.")
