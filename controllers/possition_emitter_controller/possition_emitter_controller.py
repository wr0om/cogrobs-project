"""possition_emitter_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import sys
import os
libraries_path = os.path.abspath('../my_utils')
sys.path.append(libraries_path)

from controller import Robot, Emitter, GPS
from classes_and_constans import RED, GREEN, BLUE, ORANGE, NOCOLOR, CPU_CHANNEL, NODES_CHANNEL

import math

def run_robot(robot):
    # get the time step of the current world.
    timestep = 64
    name = robot.getName()
    gps = robot.getDevice('gps')
    gps.enable(timestep)

    # Get the emitter device
    emitter = robot.getDevice('emitter')

    while robot.step(timestep) != -1:
        position = gps.getValues()
        # Send possition message
        if position is not None:
            message = (name, position)
            emitter.setChannel(CPU_CHANNEL)
            emitter.send(str(message).encode('utf-8'))


if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    run_robot(robot)

    