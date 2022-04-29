#!/usr/bin/env python

"""Package setup file.
"""

from setuptools import setup, find_packages

setup(
    name='airsim_data_collection',
    version='0.1.0',
    description='Python code for collecting data from AirSim',
    author='Adam Dai, Siddharth Tanwar',
    author_email='adamdai97@gmail.com',
    url='https://github.com/adamdai/AirSim-Data-Collection',
    packages=find_packages(),
)