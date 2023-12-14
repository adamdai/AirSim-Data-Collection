import airsim_data_collection.common.setup_path
#import airsim
import airsimdroneracinglab as airsim

import sys
import time

print("""Fly a path in an Unreal Engine environment""")

# ===================== Environment and path parameters ===================== #

drone_name = "drone_1"

waypoints = [airsim.Vector3r(0, -60, -2),
              airsim.Vector3r(-50, -60, -2),
              airsim.Vector3r(-50, 0, -2),
              airsim.Vector3r(0, 0, -2)]

# ========================== Main Code ========================== #

# Initialize airsim client
client = airsim.MultirotorClient()
client.confirmConnection()

# Initialize drone
client.enableApiControl(vehicle_name=drone_name)
client.arm(vehicle_name=drone_name)

# set default values for trajectory tracker gains
traj_tracker_gains = airsim.TrajectoryTrackerGains(
    kp_cross_track=5.0,
    kd_cross_track=0.0,
    kp_vel_cross_track=3.0,
    kd_vel_cross_track=0.0,
    kp_along_track=0.4,
    kd_along_track=0.0,
    kp_vel_along_track=0.04,
    kd_vel_along_track=0.0,
    kp_z_track=2.0,
    kd_z_track=0.0,
    kp_vel_z=0.4,
    kd_vel_z=0.0,
    kp_yaw=3.0,
    kd_yaw=0.1,
)

# Takeoff with moveOnSpline
takeoff_height = 1.0
start_position = client.simGetVehiclePose(vehicle_name=drone_name).position
takeoff_waypoint = airsim.Vector3r(
    start_position.x_val,
    start_position.y_val,
    start_position.z_val - takeoff_height,
)

client.moveOnSplineAsync(
    [takeoff_waypoint],
    vel_max=15.0,
    acc_max=5.0,
    add_position_constraint=True,
    add_velocity_constraint=False,
    add_acceleration_constraint=False,
    viz_traj=False,
    vehicle_name=drone_name,
).join()


# moveOnSpline parameters
vel_max = 10.0
acc_max = 5.0

# moveOnSpline
success = client.moveOnSplineAsync(
    waypoints,
    vel_max=vel_max,
    acc_max=acc_max,
    add_position_constraint=True,
    add_velocity_constraint=False,
    add_acceleration_constraint=False,
    viz_traj=True,
    viz_traj_color_rgba=[1.0, 0.0, 0.0, 1.0],
    vehicle_name=drone_name,
)

success.join()

print("moveOnSpline success: %r" % success)

# try:
#     pass
# except KeyboardInterrupt:
#     print("disarming...")
#     client.disarm(vehicle_name=drone_name)
#     print("resetting")
#     client.reset()
#     client.enableApiControl(False, vehicle_name=drone_name)
#     print("done.")


# print("disarming...")
# client.disarm(vehicle_name=drone_name)
# print("resetting")
# client.reset()
# client.enableApiControl(False, vehicle_name=drone_name)
# print("done.")