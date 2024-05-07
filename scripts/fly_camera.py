import numpy as np
import airsim
from airsim import Vector3r, Quaternionr, Pose

from airsim_data_collection.utility.utility import quat_to_R, R_to_quat





def generate_spiral(center, num_rings, radii, heights, num_points):
    """General spiral of camera poses in nerfstudio coordinates"""
    poses = []

    for i in range(num_rings):
        r = radii[i]
        h = heights[i]
        n = num_points[i]

        for j in range(n):
            angle = 2*np.pi*j/n
            x = r*np.cos(angle)
            y = r*np.sin(angle)
            z = h
            offset = np.array([x, y, -z])
            position = center + offset

            vx = -offset
            vx = vx / np.linalg.norm(vx)
            d = np.linalg.norm(vx[:2])
            z = vx[2]
            vz = np.array([-(z/d)*vx[0], -(z/d)*vx[1], d])
            vy = np.cross(vz, vx)
            R = np.vstack((vx, vy, vz)).T
            q = R_to_quat(R)

            poses.append((position, q))

    return poses



center = np.array([114., 150., 40.])
num_rings = 3
radii = [20, 20, 20]
heights = [10, 15, 20]
num_points = [8, 8, 8]

poses = []

for i in range(num_rings):
    r = radii[i]
    h = heights[i]
    n = num_points[i]

    for j in range(n):
        angle = 2*np.pi*j/n
        x = r*np.cos(angle)
        y = r*np.sin(angle)
        z = h
        offset = np.array([x, y, -z])
        position = center + offset

        vx = -offset
        vx = vx / np.linalg.norm(vx)
        d = np.linalg.norm(vx[:2])
        z = vx[2]
        vz = np.array([-(z/d)*vx[0], -(z/d)*vx[1], d])
        vy = np.cross(vz, vx)
        R = np.vstack((vx, vy, vz)).T
        q = R_to_quat(R)

        poses.append((position, q))


client = airsim.VehicleClient()
client.confirmConnection()

for position, q in poses:
    vehicle_pose = Pose(Vector3r(position[0], position[1], position[2]), q)
    print(vehicle_pose)
    client.simSetVehiclePose(vehicle_pose, True)
    airsim.time.sleep(0.5)

# camera_pose = Pose(Vector3r(0, 0, 0), Quaternionr(0, 0, 0, 1))  #PRY in radians
# client.simSetCameraPose(0, camera_pose)

# R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
# R = R_from_vecs(np.array([1, 0, 0]), np.array([0, 1, 0]))
# print(f"R: {R}")
# q = R_to_quat(R)

# bridge_pos = np.array([114., 150., 40.])
# camera_pos_rel = np.array([50., 50., -50.])
# camera_pos = bridge_pos + camera_pos_rel
# vx = -camera_pos_rel
# vx = vx / np.linalg.norm(vx)
# d = np.linalg.norm(vx[:2])
# z = vx[2]
# vz = np.array([-(z/d)*vx[0], -(z/d)*vx[1], d])
# print(np.linalg.norm(vz))
# vy = np.cross(vz, vx)
# R = np.vstack((vx, vy, vz)).T
# print(R)
# q = R_to_quat(R)

# vehicle_pose = Pose(Vector3r(camera_pos[0], camera_pos[1], camera_pos[2]), q)
# client.simSetVehiclePose(vehicle_pose, True)

# Get camera pose
camera_info = client.simGetCameraInfo(0)
print(f"camera position: {camera_info.pose.position}")
print(f"camera orientation: {camera_info.pose.orientation}")
# Convert orientation to matrix
R = quat_to_R(camera_info.pose.orientation)
print(f"camera rotation matrix: {R}")

# Get vehicle pose
vehicle_pose = client.simGetVehiclePose()
print(f"vehicle position: {vehicle_pose.position}")
print(f"vehicle orientation: {vehicle_pose.orientation}")



