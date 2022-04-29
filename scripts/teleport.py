"""Script to teleport a drone to target (x,y) location in the environment

"""

import airsim_data_collection.common.setup_path
import airsim
import time
import argparse
import sys
import pprint

arg_parser = argparse.ArgumentParser(description="Teleport the drone to an (x,y) location")
arg_parser.add_argument("--name", type=str, help="name of the drone to teleport")    
arg_parser.add_argument("--x", type=float, help="x coordinate")   
arg_parser.add_argument("--y", type=float, help="y coordinate")   
args = arg_parser.parse_args()    

drone = args.name

client = airsim.MultirotorClient()
client.confirmConnection()

pose = client.simGetVehiclePose()

pose.position.x_val = args.x
pose.position.y_val = args.y

client.simSetVehiclePose(pose, True, drone)