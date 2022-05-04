# AirSim-Data-Collection
Scripts for collecting data from AirSim. Python 3.8 on Windows.


## Setup

Clone the GitHub repository:

    git clone https://github.com/adamdai/AirSim-Data-Collection.git

Create conda environment from `.yml` file:

    conda env create -f environment.yml

Active the environment:
   
    conda activate airsim
   
Install `airsim_data_collection` locally from directory containing `setup.py`
   
    pip install -e .


## Settings

AirSim uses a file called `settings.json` located in the `C:\Users\USERNAME\Documents\AirSim` directory for initialization of vehicles and sensor settings. The `settings` folder in this repo contains some settings files used with different scripts (e.g. replace your `settings.json` with the contents of `settings_lidar.json` when running the `lidar_data_collection.py` script).

## Scripts

This folder contains scripts that can be run with an AirSim environment binary to spawn vehicle(s) and have them move around the environment and collect data. 
