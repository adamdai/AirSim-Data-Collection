import airsimdroneracinglab as airsim

import sys
import time

print("""Airsim drone racing fly spline""")


class DroneRacer(object):
    """
    
    """
    def __init__(self):
        """
        """
        self.drone_name = "Drone0"

    def initialize_drone(self):
        """
        """
        self.airsim_client.enableApiControl(vehicle_name=self.drone_name)
        self.airsim_client.arm(vehicle_name=self.drone_name)

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

        self.airsim_client.setTrajectoryTrackerGains(
            traj_tracker_gains, vehicle_name=self.drone_name
        )
        time.sleep(0.2)


if __name__ == "__main__":
    client = airsim.MultirotorClient()
    client.confirmConnection()

    client.arm()
    client.enableAPIControl()

    print("taking off...")
    client.takeoffAsync().join()



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


waypoints = [airsim.Vector3r(0, 60, -2),
              airsim.Vector3r(-50, 60, -2),
              airsim.Vector3r(-50, 0, -2),
              airsim.Vector3r(0, 0, -2)]

try:
    print("flying spline...")
    client.moveOnSplineAsync(
            waypoints,
            vel_max=15.0,
            acc_max=5.0,
            add_position_constraint=True,
            add_velocity_constraint=False,
            add_acceleration_constraint=False,
        ).join()

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
client.disarm()
print("resetting")
client.reset()
client.disableApiControl()
print("done.")
