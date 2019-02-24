"""@package config
Contains all config files necessary for simulator
Rignumber body configuration data

"""
from inc import log
from simulator import Simulator
from config.robot_2 import sample_robot, scene

try:
    sim = Simulator(sample_robot, scene)
except Exception as error:
    log('Error: ' + repr(error))

