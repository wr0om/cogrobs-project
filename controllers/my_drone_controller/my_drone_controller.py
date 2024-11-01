#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/

# MIT License

# Copyright (c) 2022 Bitcraze

# @file crazyflie_controllers_py.py
# Controls the crazyflie motors in webots in Python

"""crazyflie_controller_py controller."""

import sys
import os
import numpy as np
from math import cos, sin
import sys
import time
libraries_path = os.path.abspath('../my_utils')
sys.path.append(libraries_path)

from classes_and_constants import DRONE_CHANNEL, CPU_CHANNEL, ENEMY_DRONE_CHANNEL
from classes_and_constants import Location, Edge, GraphNode, Entity, Graph
from classes_and_constants import get_graph
from functions import *
from controller import Robot, Motor, InertialUnit, GPS, Gyro, Keyboard, Camera, DistanceSensor, Supervisor

sys.path.append('../../../../controllers_shared/python_based')
from pid_controller import pid_velocity_fixed_height_controller

FLYING_ATTITUDE = 1
MAX_FORWARD_SPEED = 0.5
MAX_SIDEWAY_SPEED = 0.5
MAX_YAW_RATE = 1
MAX_ALTITUDE = 2.5
SPEEDING_UNIT = 0.0005#0.005

EPSILON = 0.5

# graph = get_graph()
# got_positions_from_cpu = False
# on_node = True
# on_track = False

def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())

    ## Initialize motors
    m1_motor = robot.getDevice("m1_motor")
    m1_motor.setPosition(float('inf'))
    m1_motor.setVelocity(-1)
    m2_motor = robot.getDevice("m2_motor")
    m2_motor.setPosition(float('inf'))
    m2_motor.setVelocity(1)
    m3_motor = robot.getDevice("m3_motor")
    m3_motor.setPosition(float('inf'))
    m3_motor.setVelocity(-1)
    m4_motor = robot.getDevice("m4_motor")
    m4_motor.setPosition(float('inf'))
    m4_motor.setVelocity(1)

    ## Initialize Sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    camera = robot.getDevice("camera")
    camera.enable(timestep)
    range_front = robot.getDevice("range_front")
    range_front.enable(timestep)
    range_left = robot.getDevice("range_left")
    range_left.enable(timestep)
    range_back = robot.getDevice("range_back")
    range_back.enable(timestep)
    range_right = robot.getDevice("range_right")
    range_right.enable(timestep)

    ## Get keyboard
    keyboard = Keyboard()
    keyboard.enable(timestep)

    ## Initialize variables
    past_x_global = 0
    past_y_global = 0

    x_global = gps.getValues()[0]
    y_global = gps.getValues()[1]
    past_time = robot.getTime()

    # Crazyflie velocity PID controller
    PID_CF = pid_velocity_fixed_height_controller()
    PID_update_last_time = robot.getTime()
    sensor_read_last_time = robot.getTime()

    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)

    emitter = robot.getDevice("emitter")
    emitter.setChannel(DRONE_CHANNEL)

    def go_to_goal(x, y, z, start_flag=False):
        print(f'going to {x}, {y}, {z}')
        nonlocal x_goal, y_goal, altitude_goal
        x_goal = x
        y_goal = y
        altitude_goal = z
        execute_configuration(x_goal, y_goal, altitude_goal, start_flag)
        print(f'goal reached {x}, {y}, {z}')
    
    def stay_in_position(start_flag=False):
        nonlocal x_goal, y_goal, altitude_goal
        #print(f'staying in position {x_goal}, {y_goal}, {altitude_goal}')
        x_goal = gps.getValues()[0]
        y_goal = gps.getValues()[1]
        altitude_goal = gps.getValues()[2]
        starting_time = robot.getTime()
        while robot.getTime() - starting_time < 0.5:
            execute_configuration(x_goal, y_goal, altitude_goal, start_flag)
        execute_configuration(x_goal, y_goal, altitude_goal, start_flag)
        #print('finished staying in position')

    def execute_configuration(x_goal, y_goal, altitude_goal, start_flag=False):

        nonlocal past_time, past_x_global, past_y_global, height_desired, x_global, y_global

        # Main loop executing the commands:
        forward_desired = 0
        sideways_desired = 0
        yaw_desired = 0
        height_diff_desired = 0

        reached_goal = False
        while robot.step(timestep) != -1:
            dt = robot.getTime() - past_time
            actual_state = {}

            ## Get sensor data
            roll = imu.getRollPitchYaw()[0]
            pitch = imu.getRollPitchYaw()[1]
            yaw = imu.getRollPitchYaw()[2]
            yaw_rate = gyro.getValues()[2]
            altitude = gps.getValues()[2]
            x_global = gps.getValues()[0]
            y_global = gps.getValues()[1]

            if not start_flag:
                # send the current location to the cpu
                current_location = gps.getValues()
                message = (robot_name, current_location)
                send_msg_to_cpu(emitter, message)

            # Calculate global velocities
            v_x_global = (x_global - past_x_global) / dt
            v_y_global = (y_global - past_y_global) / dt

            ## Get body fixed velocities
            cosyaw = cos(yaw)
            sinyaw = sin(yaw)
            v_x = v_x_global * cosyaw + v_y_global * sinyaw
            v_y = -v_x_global * sinyaw + v_y_global * cosyaw

            ## Initialize values
            initial_state = {"pos": np.array([x_global, y_global, altitude]), "moment": np.array([v_x, v_y, yaw])}
            desired_state = {"pos": np.array([x_goal, y_goal, altitude_goal]), "moment": np.array([0, 0, 0])}

            desired_direction = desired_state["pos"] - initial_state["pos"]
            if np.linalg.norm(desired_direction) != 0:
                desired_direction = desired_direction / np.linalg.norm(desired_direction)

            distance = np.linalg.norm(initial_state["pos"] - desired_state["pos"])

            if distance < EPSILON:
                reached_goal = True
                
            
            forward_distance = desired_state["pos"][0] - initial_state["pos"][0]
            sideways_distance = desired_state["pos"][1] - initial_state["pos"][1]

            # print("forward_distance: ", forward_distance , "sideways_distance: ", sideways_distance)
            

            # print(f"Current position: {initial_state['pos']}")
            # print(f"Desired position: {desired_state['pos']}")
            # # print(f"Current velocity: {initial_state['moment']}")
            # # print(f"Desired velocity: {desired_state['moment']}")
            # # print("\n")
            # # print(f"deseired_direction: {desired_direction}")
            # print(f"distance: {distance}")

            slowing_forward = False
            slowing_sideways = False

            if np.linalg.norm(initial_state["pos"] - desired_state["pos"]) > 0.1:
                #print("moving")
                if not slowing_forward:
                    if forward_distance > SPEEDING_UNIT and forward_desired < MAX_FORWARD_SPEED:
                        forward_desired += SPEEDING_UNIT
                    elif forward_distance < -SPEEDING_UNIT and forward_desired > -MAX_FORWARD_SPEED:
                        forward_desired -= SPEEDING_UNIT

                if not slowing_sideways:
                    if sideways_distance > SPEEDING_UNIT and sideways_desired < MAX_SIDEWAY_SPEED:
                        sideways_desired += SPEEDING_UNIT
                    elif sideways_distance < -SPEEDING_UNIT and sideways_desired > -MAX_SIDEWAY_SPEED:
                        sideways_desired -= SPEEDING_UNIT

                if yaw_desired - yaw > 0.1:
                    yaw_desired += 1
                elif yaw_desired - yaw < -0.1:
                    yaw_desired -= 1

                if altitude_goal - altitude > 0.1:
                    height_diff_desired = 0.1
                elif altitude_goal - altitude < -0.1 :
                    height_diff_desired = -0.1

                forward_desired = np.sign(forward_desired)*min(MAX_FORWARD_SPEED, np.abs(forward_desired))
                sideways_desired = np.sign(sideways_desired)*min(MAX_SIDEWAY_SPEED, np.abs(sideways_desired))

                if np.linalg.norm(initial_state["pos"][0] - desired_state["pos"][0]) < 0.3 and np.abs(forward_desired) > 10*SPEEDING_UNIT:
                    forward_desired -= np.sign(forward_desired)*SPEEDING_UNIT
                    slowing_forward = True
                    #print("slowing forward")
                
                if np.linalg.norm(initial_state["pos"][1] - desired_state["pos"][1]) < 0.3 and np.abs(sideways_desired) > 10*SPEEDING_UNIT:
                    slowing_sideways = True
                    sideways_desired -= np.sign(sideways_desired)*SPEEDING_UNIT
        
            else:
                #print("slowing down")
                if np.abs(forward_desired) > SPEEDING_UNIT:
                    forward_desired -= np.sign(forward_desired)*SPEEDING_UNIT
                else:
                    forward_desired = 0

                if np.abs(sideways_desired) > SPEEDING_UNIT:
                    sideways_desired -= np.sign(sideways_desired)*SPEEDING_UNIT
                else:
                    sideways_desired = 0

                # if yaw_desired - yaw > 0.1:
                #     yaw_desired += 0.1
                # elif yaw_desired - yaw < -0.1:
                #     yaw_desired -= 0.1
                # else:
                #     yaw_desired = 0
                
                # if altitude_goal - altitude > 0.1:
                #     height_diff_desired = 0.1
                # elif altitude_goal - altitude < -0.1:
                #     height_diff_desired = -0.1
                # else:
                #     height_diff_desired = 0

                if forward_desired == sideways_desired == 0:
                    reached_goal = True

            height_desired += height_diff_desired * dt

            #print(f"forward_desired: {forward_desired}, sideways_desired: {sideways_desired}, yaw_desired: {yaw_desired}, height_desired: {height_desired}")

            ## Example how to get sensor data
            ## range_front_value = range_front.getValue()
            ## cameraData = camera.getImage()

            ## PID velocity controller with fixed height
            nonlocal PID_CF
            motor_power = PID_CF.pid(dt, forward_desired, sideways_desired,
                                    yaw_desired, height_desired,
                                    roll, pitch, yaw_rate,
                                    altitude, v_x, v_y)

            m1_motor.setVelocity(-motor_power[0])
            m2_motor.setVelocity(motor_power[1])
            m3_motor.setVelocity(-motor_power[2])
            m4_motor.setVelocity(motor_power[3])

            past_time = robot.getTime()
            past_x_global = x_global
            past_y_global = y_global

            if reached_goal:
                return

    def go_to(node_name):
        node = graph.get_node(node_name)
        go_to_goal(node.fisical_position[0], node.fisical_position[1], MAX_ALTITUDE)

    def get_enemy_drones_positions(enemy_drone_names, enemy_drone_robots):
        """
        Get the positions of enemy drones in the environment.

        Parameters:
        enemy_drone_names: List of names of enemy drones.
        enemy_drone_robots: List of robot objects representing enemy drones.

        Returns:
        A list of tuples containing the positions of enemy drones.
        """
        enemy_drone_positions = dict()
        for enemy_name, enemy_robot in zip(enemy_drone_names, enemy_drone_robots):
            enemy_position = enemy_robot.getField("translation").getSFVec3f()
            enemy_drone_positions[enemy_name] = enemy_position
        return enemy_drone_positions
    
    def lose_control():
        m1_motor.setVelocity(0)
        m2_motor.setVelocity(0)
        m3_motor.setVelocity(0)
        m4_motor.setVelocity(0)
    
    global got_positions_from_cpu
    altitude_goal = -1

    robot_name = robot.getName()
    print(f"Robot name: {robot_name}")


    # set the channel to communicate with the cpu
    if robot_name == "Drone":
        drone_channel = DRONE_CHANNEL
        altitude_goal = MAX_ALTITUDE # Set your target altitude
    else:
        drone_channel = ENEMY_DRONE_CHANNEL
        supervisor = Supervisor()
        # get the drone robot
        drone_robot = supervisor.getFromDef("Drone")

        if "1" in robot_name:
            altitude_goal = 1
        elif "2" in robot_name:
            altitude_goal = 2
        elif "3" in robot_name:
            altitude_goal = 3
        elif "4" in robot_name:
            altitude_goal = 4


    receiver.setChannel(drone_channel)


    # lifting off
    while robot.step(timestep) != -1 and gps.getValues()[2] is None:
        pass

    current_location = gps.getValues()
    x_goal = current_location[0]
    y_goal = current_location[1]
    height_desired = current_location[2]
    start_flag = True
    go_to_goal(x_goal, y_goal, altitude_goal, start_flag)
    start_flag = False
    print('finished lifting off')

    # send the current location to the cpu
    current_location = gps.getValues()
    message = (robot_name, current_location)
    print(f"Sending msg to CPU: {message}")
    send_msg_to_cpu(emitter, message)

    
    last_time = time.time()
    TIME_TO_SEND = 0.5
    while robot.step(timestep) != -1:
        current_location = gps.getValues()
        current_time = time.time()
        # Send the current location to the cpu every 1 seconds
        if current_time - last_time > TIME_TO_SEND:
            message = (robot_name, current_location)
            send_msg_to_cpu(emitter, message)
            last_time = current_time

        if robot_name != "Drone":
            # Check if Drone is about to crash into you
            drone_position = get_enemy_drones_positions(["Drone"], [drone_robot])["Drone"]
            distance_from_drone = np.linalg.norm(np.array(drone_position) - np.array(current_location))
            if distance_from_drone < EPSILON:
                print(f"Drone is about to crash into {robot_name}")
                # enemy drone loses control
                lose_control()
                break



        # Check for incoming packets
        if receiver.getQueueLength() > 0:
            messages = get_msg_from_cpu(receiver)
            print(f"Messages {robot_name} got: {messages}")
            # get last message sent by the CPU, reverse the list to get the last message first
            messages.reverse()
            for message in messages:
                if message[0] == "CPU":
                    print(f"Received message from CPU: {message}")
                    # get the goal location from the CPU
                    x_goal, y_goal, altitude_goal = message[1]
                    distance = np.linalg.norm(np.array([x_goal, y_goal, altitude_goal]) - np.array(current_location))
                    # only go to the new goal location if the distance is greater than EPSILON
                    if distance > EPSILON:
                        print(f"{robot_name} received goal location: {x_goal}, {y_goal}, {altitude_goal}")
                        go_to_goal(x_goal, y_goal, altitude_goal, start_flag)
        else:
            stay_in_position(start_flag)


    # while robot.step(timestep) != -1:
    #     stay_in_position()

    # robot.cleanup()


if __name__ == '__main__':
    robot = Robot()
    run_robot(robot)
    