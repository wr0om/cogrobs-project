"""odometer_calculation controller."""


# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import sys
import os
libraries_path = os.path.abspath('../my_utils')
sys.path.append(libraries_path)

from controller import Robot, Camera, Receiver
import math
from classes_and_constans import RED, GREEN, BLUE, ORANGE, NOCOLOR, WAITER_CHANNEL, CPU_CHANNEL
from classes_and_constans import Location, Edge, GraphNode, Entity, Graph
from classes_and_constans import get_graph
from functions import get_positions_graph_from_cpu
import numpy as np

import math
import ast

graph = get_graph()
got_positions_from_cpu = False
on_node = True
on_track = False

FRONT = 0
BACK = 3.14
SIDE = 1.57

def get_color_from_camera(image, width, height):
    # Process the image to detect color
    red_count = 0
    green_count = 0
    blue_count = 0
    orange_count = 0
    for x in range(width):
        for y in range(height):
            r = Camera.imageGetRed(image, width, x, y)
            g = Camera.imageGetGreen(image, width, x, y)
            b = Camera.imageGetBlue(image, width, x, y)
            if r > 200 and g < 100 and b < 100:  # Detect red color
                red_count += 1
            elif g > 200 and r < 100 and b < 100:  # Detect green color
                green_count += 1
            elif b > 200 and r < 100 and g < 100:  # Detect blue color
                blue_count += 1
            elif r > 200 and g > 100 and b < 100:  # Detect orange color
                orange_count += 1

    if red_count > (width * height * 0.1):  # If more than 10% of the image is red
        # print("Red color detected")
        return RED
    elif green_count > (width * height * 0.1):  # If more than 10% of the image is green
        # print("Green color detected")
        return GREEN
    elif blue_count > (width * height * 0.1):  # If more than 10% of the image is blue
        # print("Blue color detected")
        return BLUE
    elif orange_count > (width * height * 0.1):  # If more than 10% of the image is orange
        # print("Orange color detected")
        return ORANGE
    else:
        # print("No color detected")
        return NOCOLOR

def update_sensor_data(left_ps, right_ps, last_ps_values, ps_values, dis_values, encoder_unit, distance_between_wheels, robot_pos):

    # Read the sensors:
    ps_values[0] = left_ps.getValue()
    ps_values[1] = right_ps.getValue()

    print("----------------------------")
    print("Left ps: ", ps_values[0], "Right ps: ", ps_values[1])

    for ind in range(2):
        diff = ps_values[ind] - last_ps_values[ind]
        if diff < 0.001:
            diff = 0 # to avoid noise
            ps_values[ind] = last_ps_values[ind]

        dis_values[ind] = diff * encoder_unit

    # compute linear and angular velocities
    v = (dis_values[0] + dis_values[1]) / 2.0 # linear velocity
    w = (dis_values[1] - dis_values[0]) / distance_between_wheels  # angular velocity

    dt = 1
    robot_pos[2] += w * dt

    vx = v * math.cos(robot_pos[2])
    vy = v * math.sin(robot_pos[2])

    robot_pos[0] += vx * dt
    robot_pos[1] += vy * dt

    print("Robot position: ", robot_pos)


    print("Left distance: ", dis_values[0], "Right distance: ", dis_values[1])

    for ind in range(2):
        last_ps_values[ind] = ps_values[ind]

def make_turn(robot, angle, left_motor, right_motor, max_speed, wheel_radius, distance_between_wheels, timestep):
    leaniar_velocity = wheel_radius * max_speed

    start_time = robot.getTime()

    
    angle_of_rotation = angle
    rate_of_rotation = (2*leaniar_velocity) / distance_between_wheels
    duration_turn = abs(angle_of_rotation / rate_of_rotation)

    rot_start_time = start_time
    rot_end_time = rot_start_time + duration_turn


    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    turn_completed = False

    while robot.step(timestep) != -1 and not turn_completed:
        current_time = robot.getTime()

        if current_time < rot_end_time:
            left_speed = -max_speed*np.sign(angle)
            right_speed = max_speed*np.sign(angle)
        else:
            turn_completed = True
            left_speed = 0
            right_speed = 0
     
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

def go_forward(robot, distance, left_motor, right_motor, max_speed, wheel_radius, distance_between_wheels, timestep):
    vanila_speed = max_speed*0.25
    leaniar_velocity = wheel_radius * vanila_speed

    duration_side = abs(distance / leaniar_velocity)

    start_time = robot.getTime()

    lin_end_time = start_time + duration_side

    completed = False
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1 and not completed:
        current_time = robot.getTime()

        if current_time < lin_end_time:
            left_speed = vanila_speed
            right_speed = vanila_speed
        else:
            completed = True
            left_speed = 0
            right_speed = 0
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

def go_from_node_to_node(robot, start_node, end_node, front_gps, back_gps, left_motor,
                          right_motor, max_speed, wheel_radius, distance_between_wheels, timestep, left_ir, right_ir, camera):
    if front_gps is None or back_gps is None:
        return
    
    
    currnet_color = get_color_from_camera(camera.getImage(), camera.getWidth(), camera.getHeight())
    print("Current color: ", currnet_color)

    rotations = [FRONT, SIDE, BACK]

    go_forward(robot, 0.2, left_motor, right_motor, max_speed, wheel_radius, distance_between_wheels, timestep)

    start_node = graph.get_node(start_node)
    end_node = graph.get_node(end_node)

    print("Start node position: ", start_node.fisical_position)
    print("start node neighbors: ", [n.name for n in start_node.get_neighbours()])
    print("End node position: ", end_node.fisical_position)
    print("Robot position: ", np.array(back_gps.getValues()))

    robot_direction = np.array(front_gps.getValues()[:2]) - np.array(back_gps.getValues()[:2])
    robot_direction = robot_direction / np.linalg.norm(robot_direction)
    robot_pos = np.array(back_gps.getValues()[:2])

    goal_direction = np.array(end_node.fisical_position[:2]) - np.array(start_node.fisical_position[:2])
    goal_direction = goal_direction / np.linalg.norm(goal_direction)

    print("Robot direction: ", robot_direction)
    print("Goal direction: ", goal_direction)


    angle = np.arccos(np.dot(robot_direction, goal_direction)/(np.linalg.norm(robot_direction)*np.linalg.norm(goal_direction)))
    print("Angle: ", angle)
    closest_angle = min([rot for rot in rotations], key=lambda x: abs(x - angle))

    if len(start_node.get_neighbours()) == 2 and closest_angle == SIDE:
        closest_angle = FRONT
    if len(start_node.get_neighbours()) == 3 and closest_angle == SIDE and len(start_node.name) >= 2 and len(end_node.name) >= 2 and start_node.name[:2] == end_node.name[:2]:
        closest_angle = FRONT

    # find direction of left/right rotation
    if np.cross(robot_direction, goal_direction) >= 0:
        closest_angle = -closest_angle
    print("Closest Angle: ", closest_angle)
    if angle > 0.1:
        make_turn(robot, closest_angle, left_motor, right_motor, max_speed, wheel_radius, distance_between_wheels, timestep)
    go_forward(robot, 0.2, left_motor, right_motor, max_speed, wheel_radius, distance_between_wheels, timestep)
    reached_other_node = False
    while robot.step(timestep) != -1 and not reached_other_node:
        color = get_color_from_camera(camera.getImage(), camera.getWidth(), camera.getHeight())

        print("Color: ", color)
        if color!= currnet_color and color != NOCOLOR:
            reached_other_node = True
            left_speed = 0
            right_speed = 0
        else:
            left_ir_value = left_ir.getValue()
            right_ir_value = right_ir.getValue()

            print("Left IR: ", left_ir_value, "Right IR: ", right_ir_value)

            if (left_ir_value > right_ir_value) and (700 < left_ir_value <= 1000):
                print("turn right")
                left_speed = -max_speed
                right_speed = max_speed
            elif (right_ir_value > left_ir_value) and (700 < right_ir_value <= 1000):
                print("turn left")
                left_speed = max_speed
                right_speed = -max_speed
            else:
                left_speed = max_speed
                right_speed = max_speed

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
    
    print("Reached other node")

def dont_move(left_motor, right_motor):
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

def run_robot(robot):
    # get the time step of the current world.
    timestep = 64
    max_speed = -6.28*0.2

    # Create an instances of motors
    left_motor = robot.getDevice('motor_1')
    right_motor = robot.getDevice('motor_2')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    # get position sensor instances
    left_ps = robot.getDevice('ps_1')
    left_ps.enable(timestep)
    right_ps = robot.getDevice('ps_2')
    right_ps.enable(timestep)

    # get camera instance
    camera = robot.getDevice('color_sensor')
    camera.enable(timestep)

    # get ir sensor instances
    left_ir = robot.getDevice('ir_w_1')
    left_ir.enable(timestep)

    right_ir = robot.getDevice('ir_w_2')
    right_ir.enable(timestep)

    # Get the receiver device
    receiver = robot.getDevice('receiver')
    receiver.enable(timestep)

    # Get the emitter device
    emitter = robot.getDevice('emitter')

    # get gps instance
    front_gps = robot.getDevice('front_gps')
    front_gps.enable(timestep)
    back_gps = robot.getDevice('back_gps')
    back_gps.enable(timestep)

    # compute encoder unit
    wheel_radius = 0.4
    distance_between_wheels = 0.6

    wheel_circumference = 2.0 * 3.14 * wheel_radius
    encoder_unit = wheel_circumference / 6.28

    # robot position
    robot_pos = [0, 0, 0] # x, y, theta
    last_ps_values = [0, 0]
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller

    ps_values = [0, 0]
    dis_values = [0, 0]

    finnished = False
    while robot.step(timestep) != -1:
        # Check for incoming packets
        print(f"Queue length: {receiver.getQueueLength()}")

        global got_positions_from_cpu

        if not got_positions_from_cpu:
            got_positions_from_cpu = get_positions_graph_from_cpu(receiver, emitter, graph, got_positions_from_cpu)

        path = ["ml_2", "mr_1", "br_1", "bl_2", "ml_2"]
        if got_positions_from_cpu and not finnished:
            for i in range(len(path) - 1):
                go_from_node_to_node(robot, path[i], path[i+1], front_gps, back_gps, left_motor, right_motor, max_speed, wheel_radius, distance_between_wheels, timestep, left_ir, right_ir, camera)
            finnished = True

        update_sensor_data(left_ps, right_ps, last_ps_values, ps_values, dis_values, encoder_unit, distance_between_wheels, robot_pos)

        
        
       


        

    # Enter here exit cleanup code.

if __name__ == "__main__":
    # create the Robot instance.
    # robot = Robot()
    # run_robot(robot)
    pass

    
