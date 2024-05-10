import os
import numpy as np
import airsim
from airsim import Vector3r, Quaternionr, Pose
import plotly.graph_objects as go
import cv2 as cv
import json
import shutil

from airsim_data_collection.utility.utility import quat_to_R, R_to_quat


def generate_spiral(center, num_rings, radii, heights, num_points):
    """General spiral of camera poses in nerfstudio coordinates"""
    poses = []
    transforms = []  # camera transform matrices for nerfstudio

    for i in range(num_rings):
        r = radii[i]
        z = heights[i]
        n = num_points[i]

        for j in range(n):
            # AirSim coordinates
            angle = 2*np.pi*j/n
            x = r*np.cos(angle)
            y = r*np.sin(angle)
            offset = np.array([x, y, -z])
            position = center + offset

            vx = -offset
            vx = vx / np.linalg.norm(vx)
            d = np.linalg.norm(vx[:2])
            unit_z = vx[2]
            vz = np.array([-(unit_z/d)*vx[0], -(unit_z/d)*vx[1], d])
            vy = np.cross(vz, vx)
            R = np.vstack((vx, vy, vz)).T
            q = R_to_quat(R)

            poses.append((position, q))

            # Nerfstudio coordinates
            c2w = np.eye(4)
            ns_pos = np.array([y, x, z])  # +Z is back and away from camera (opposite look-direction)
            vz = ns_pos / np.linalg.norm(ns_pos)  # normalize
            d = np.linalg.norm(vz[:2])  # distance in x-y plane
            unit_z = vz[2]              # z component of unit vector
            vy = np.array([-(unit_z/d)*vz[0], -(unit_z/d)*vz[1], d])  # y is in x-y plane
            vx = np.cross(vy, vz)
            c2w[:3, :3] = np.vstack((vx, vy, vz)).T
            c2w[:3, 3] = ns_pos
            transforms.append(c2w)

    return poses, transforms


if __name__ == '__main__':

    # Spiral parameters
    center = np.array([99., -449., -57.])
    num_rings = 3
    radii = [250, 300, 400]
    heights = [150, 180, 200]
    num_points = [20, 20, 20]

    poses, transforms = generate_spiral(center, num_rings, radii, heights, num_points)

    # Plot the spiral
    # fig = go.Figure()
    # for position, q in poses:
    #     fig.add_trace(go.Scatter3d(x=[position[0]], y=[position[1]], z=[position[2]], mode='markers', marker=dict(size=5, color='red')))
    # fig.update_layout(scene=dict(aspectmode='data'))
    # fig.show()

    # Folders for saving data
    data_folder = '../data/spiral'
    # Check if folder exists
    if os.path.exists(data_folder):
        print("Data folder exists, deleting...")
        shutil.rmtree(data_folder)
    os.makedirs(f'{data_folder}/images')
    frames = []

    client = airsim.VehicleClient()
    client.confirmConnection()

    for i, (position, q) in enumerate(poses):
        vehicle_pose = Pose(Vector3r(position[0], position[1], position[2]), q)
        client.simSetVehiclePose(vehicle_pose, True)
        
        # Capture image
        image = client.simGetImage(0, airsim.ImageType.Scene)
        image = cv.imdecode(np.frombuffer(image, np.uint8), -1)
        img_path = f'images/{i}.png'
        cv.imwrite(f'{data_folder}/{img_path}', image)

        # Save camera pose
        frame = {
            "file_path": img_path,
            "transform_matrix": transforms[i].tolist(),
            "colmap_im_id": i,
        }
        frames.append(frame)
        
        #airsim.time.sleep(0.01)
    
    # Generate transforms.json
    # TODO: pull camera params from settings.json
    W, H = 1280, 720
    FOV = 90
    fl = W / (2 * np.tan(np.radians(FOV) / 2))
    out = {
        "w": W,
        "h": H,
    }
    
    out["fl_x"] = fl
    out["fl_y"] = fl
    out["cx"] = W/2
    out["cy"] = H/2
    out["k1"] = 0.0
    out["k2"] = 0.0
    out["p1"] = 0.0
    out["p2"] = 0.0
        
    out["frames"] = frames

    print("Saving...", os.path.join(data_folder, 'transforms.json'))
    with open(os.path.join(data_folder, 'transforms.json'), 'w', encoding="utf-8") as f: 
        json.dump(out, f, indent=4)




