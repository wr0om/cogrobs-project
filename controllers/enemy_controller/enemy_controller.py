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
from numpy import random
import sys
import os
import numpy as np
from math import cos, sin
import sys
libraries_path = os.path.abspath('../my_utils')
sys.path.append(libraries_path)

from classes_and_constants import *
from functions import *
from controller import Robot, Keyboard, Supervisor

sys.path.append('../../../../controllers_shared/python_based')
from pid_controller import pid_velocity_fixed_height_controller

FLYING_ATTITUDE = 1
MAX_FORWARD_SPEED = 0.5#0.5
MAX_SIDEWAY_SPEED = 0.5#0.5
MAX_YAW_RATE = 1
MAX_ALTITUDE = 2.5
SPEEDING_UNIT = 0.005#0.005


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

    if OBSTACLES:
        ## Initialize distance sensors
        ps0 = []
        psNames0 = [
            'ds0', 'ds1', 'ds2', 'ds3',
            'ds4', 'ds5', 'ds6', 'ds7'
        ]
        for i in range(8):
            ps0.append(robot.getDevice(psNames0[i]))
            ps0[i].enable(timestep)
        
        ps1 = []
        psNames1 = [
            'ds00', 'ds11', 'ds22', 'ds33',
            'ds44', 'ds55', 'ds66', 'ds77'
        ]
        for i in range(8):
            ps1.append(robot.getDevice(psNames1[i]))
            ps1[i].enable(timestep)
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

    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)

    # emitter = robot.getDevice("emitter")
    # emitter.setChannel(CPU_CHANNEL)


    def go_to_goal(x, y, z, adverserial):
        # print(f'going to {x}, {y}, {z}')
        nonlocal x_goal, y_goal, altitude_goal
        x_goal = x
        y_goal = y
        altitude_goal = z
        execute_configuration(x_goal, y_goal, altitude_goal, adverserial)
        # print(f'goal reached {x}, {y}, {z}')
    
    def stay_in_position(robot_name, adverserial):
        # if robot_name == "Drone":
        #     print('staying in position')
        nonlocal x_goal, y_goal, altitude_goal
        #print(f'staying in position {x_goal}, {y_goal}, {altitude_goal}')
        x_goal = gps.getValues()[0]
        y_goal = gps.getValues()[1]
        altitude_goal = gps.getValues()[2]
        starting_time = robot.getTime()
        while robot.getTime() - starting_time < 0.2:
            execute_configuration(x_goal, y_goal, altitude_goal, adverserial)
        execute_configuration(x_goal, y_goal, altitude_goal, adverserial)
        #print('finished staying in position')

    def execute_configuration(x_goal, y_goal, altitude_goal, adverserial):

        nonlocal past_time, past_x_global, past_y_global, height_desired, x_global, y_global
        # Main loop executing the commands:
        forward_desired = 0
        sideways_desired = 0
        yaw_desired = 0
        height_diff_desired = 0

        reached_goal = False
        time_change = robot.getTime()
        moving_direction_clear = True
        x_save_goal, y_save_goal = x_goal, y_goal
        first_time = True
        stops = False
        if OBSTACLES:
            prev_obstacle_values0 = [p.getValue() for p in ps0]
            prev_obstacle_values1 = [p.getValue() for p in ps1]
        prev_new = [False for _ in range(8)]
        encontered = [False for _ in range(8)]
        start_time = robot.getTime()
        time_to_reset = robot.getTime()
        reseting = False
        pool = True
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


            drone_position = get_enemy_drones_positions(["Drone"], [drone_robot])["Drone"]
            vec = np.array([x_global, y_global, altitude]) - np.array(drone_position)
            distance_from_drone = np.linalg.norm(vec)
            if distance_from_drone < EPSILON:
                print(f"Drone is about to crash into {robot_name}")
                # enemy drone loses control
                lose_control(supervisor=supervisor)
                break

            #obstacle avoidance
            final_eq_goal = x_goal == x_save_goal and y_goal == y_save_goal
            if OBSTACLES:          #obstacle avoidance
                moving_direction_vector = np.array([x_goal, y_goal]) - np.array([x_global, y_global])
                goal_direction_vector = np.array([x_save_goal, y_save_goal]) - np.array([x_global, y_global])
                moving_angle = np.arctan2(moving_direction_vector[1], moving_direction_vector[0])
                obstacle_values0  = [p.getValue()  for p in ps0]
                obstacle_values1  = [p.getValue()  for p in ps1]
                def_values0 = [pp - p for p, pp in zip(obstacle_values0, prev_obstacle_values0)]
                def_values1 = [pp - p for p, pp in zip(obstacle_values1, prev_obstacle_values1)]
                detected_obstacle0  = [(10 < d ) and p < 700.0   for p, d in zip(obstacle_values0, def_values0)]
                detected_obstacle1  = [p < 600.0   for p, d in zip(obstacle_values1, def_values1)]
                detected_obstacle_x  = [ p < 600.0 and d > 1 for p, d in zip(obstacle_values0, def_values0)]
                near_obstacle = [(6 < d and p < 600.0)  for p, d in zip(obstacle_values1, def_values1)]
                found_new  = find_new_obstacle1(obstacle_values0, detected_obstacle0, detected_obstacle1, def_values0, prev_new)
                # found_new  = [not(prev_new[(i-1)%8] or prev_new[(i +1)%8]) and (detected_obstacle[i] and(detected_obstacle[(i-1)%8] or detected_obstacle[(i+1)%8])) for i in range(8)]
                remove_obstacle = [prev_new[i] and (obstacle_values0[i] > 850) for i in range(8)]
                final_eq_goal = x_goal == x_save_goal and y_goal == y_save_goal
                norm = np.linalg.norm(moving_direction_vector)
                if any(found_new) and not stops:
                    print('obstacle detected stopping')
                    x_goal, y_goal = x_global, y_global
                    # if any([p< 300.0 for p in obstacle_values0]):
                    #     x_goal, y_goal = [x_global, y_global] - 0.3 * moving_direction_vector
                    stops = True
                elif reached_goal and stops:
                    print('moving left')
                    vector_angle = moving_angle + np.pi/2 if pool else moving_angle - np.pi/2
                    moving_obs_vector = np.array([np.cos(vector_angle), np.sin(vector_angle)]) + np.array([x_global, y_global])
                    x_goal, y_goal = moving_obs_vector
                    reached_goal = False
                    stops = False
                    print(x_goal, y_goal)
                elif all([p==False for p in prev_new]) and reached_goal and not stops:
                    x_goal, y_goal = x_save_goal, y_save_goal
                    reached_goal = False
                elif any(detected_obstacle_x) and not stops and (robot.getTime() - start_time) > 2:
                    print('getting closer to wall')
                    i = detected_obstacle_x.index(True)
                    if not detected_obstacle_x[(i+1)%8]:
                        moving_from_wall = moving_angle + np.pi/4 
                    elif not detected_obstacle_x[(i-1)%8]:
                        moving_from_wall = moving_angle - np.pi/4
                    else:
                        moving_from_wall = moving_angle + np.pi/2 if pool else moving_angle - np.pi/2 
                    x_goal, y_goal = np.array([np.cos(moving_from_wall), np.sin(moving_from_wall)]) + np.array([x_global, y_global]) - 0.3 * moving_direction_vector
                    start_time = robot.getTime()
                elif (norm < 0.3) and not final_eq_goal and not stops and any(prev_new):
                    x_goal, y_goal = np.array([np.cos(vector_angle), np.sin(vector_angle)]) + np.array([x_global, y_global])

                # if any(remove_obstacle):
                #     x_goal, y_goal = x_save_goal, y_save_goal
                prev_new = [(p or q) and not t for p, q, t in zip(prev_new, found_new, remove_obstacle)]
                encontered = [p or q for p, q in zip(encontered, found_new)]
                prev_obstacle_values0 = obstacle_values0
                prev_obstacle_values1 = obstacle_values1
                          
            if adverserial and (not OBSTACLES or not stops):
                if distance_from_drone < 2.5 * EPSILON:
                    #print(f"running away")
                    vec = vec / distance_from_drone
                    x_goal = (vec[0] + x_global) * 0.5 + x_goal * 0.5
                    y_goal = (vec[1] + y_global) * 0.5 + y_goal * 0.5
                    

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
            final_state = {"pos": np.array([x_save_goal, y_save_goal, altitude_goal]), "moment": np.array([0, 0, 0])}

            desired_direction = desired_state["pos"] - initial_state["pos"]
            if np.linalg.norm(desired_direction) != 0:
                desired_direction = desired_direction / np.linalg.norm(desired_direction)

            distance = np.linalg.norm(initial_state["pos"] - desired_state["pos"])

            
            forward_distance = desired_state["pos"][0] - initial_state["pos"][0]
            sideways_distance = desired_state["pos"][1] - initial_state["pos"][1]

            slowing_forward = False
            slowing_sideways = False

            if np.linalg.norm(initial_state["pos"] - desired_state["pos"]) > 0.1:
                #print("moving")
                env =  1.0 if not OBSTACLES  else ((min(obstacle_values0) + 100) / 1100.0)
                change = 0.65 if np.abs(altitude_goal - altitude) > 0.1 else 1.0
                if stops:
                    env = 1.0
                    change = 1.0
                if not slowing_forward:
                    if forward_distance > change * env * SPEEDING_UNIT and forward_desired < env * MAX_FORWARD_SPEED:
                         forward_desired = np.clip(forward_desired + change * env *SPEEDING_UNIT,-env * MAX_FORWARD_SPEED, env * MAX_FORWARD_SPEED)
                    elif forward_distance < -change *SPEEDING_UNIT and forward_desired > -env  * MAX_FORWARD_SPEED:
                        forward_desired = np.clip(forward_desired - change * env *SPEEDING_UNIT,-env * MAX_FORWARD_SPEED, env * MAX_FORWARD_SPEED)
                        
                if altitude_goal - altitude > 0.1:
                    height_diff_desired = 0.1

                if altitude_goal - altitude < -0.1 :
                    height_diff_desired = -0.1
                if not slowing_sideways:
                    if sideways_distance > change *SPEEDING_UNIT and sideways_desired < env * MAX_SIDEWAY_SPEED:
                        sideways_desired = np.clip(sideways_desired + change  * env * SPEEDING_UNIT,-env * MAX_SIDEWAY_SPEED, env * MAX_SIDEWAY_SPEED)
                    elif sideways_distance < -change *SPEEDING_UNIT and sideways_desired > -env * MAX_SIDEWAY_SPEED:
                        sideways_desired = np.clip(sideways_desired - change  * env * SPEEDING_UNIT,-env * MAX_SIDEWAY_SPEED, env * MAX_SIDEWAY_SPEED)

                if yaw_desired - yaw > 0.1:
                    yaw_desired += 1
                elif yaw_desired - yaw < -0.1:
                    yaw_desired -= 1

                forward_desired = np.sign(forward_desired)*min(env * MAX_FORWARD_SPEED, np.abs(forward_desired))
                sideways_desired = np.sign(sideways_desired)*min(env * MAX_SIDEWAY_SPEED, np.abs(sideways_desired))

                if np.linalg.norm(initial_state["pos"][0] - desired_state["pos"][0]) < 0.3 and np.abs(forward_desired) > 10*SPEEDING_UNIT:
                    forward_desired -= np.sign(forward_desired)*SPEEDING_UNIT
                    slowing_forward = True
                    #print("slowing forward")
                
                if np.linalg.norm(initial_state["pos"][1] - desired_state["pos"][1]) < 0.3 and np.abs(sideways_desired) > 10*SPEEDING_UNIT:
                    slowing_sideways = True
                    sideways_desired -= np.sign(sideways_desired)*change * env *SPEEDING_UNIT
        
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

                if forward_desired == sideways_desired == 0:
                    reached_goal = True

            height_desired += height_diff_desired * dt

            #print(f"forward_desired: {forward_desired}, sideways_desired: {sideways_desired}, yaw_desired: {yaw_desired}, height_desired: {height_desired}")

            ## Example how to get sensor data
            ## range_front_value = range_front.getValue()
            ## cameraData = camera.getImage()

            ## PID velocity controller with fixed height
            nonlocal PID_CF
            # if robot_name[-1] == "3":
            #     print(f"forward_desired: {forward_desired}, sideways_desired: {sideways_desired}, yaw_desired: {yaw_desired}, height_desired: {height_desired}")
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

            if reached_goal and np.linalg.norm(initial_state["pos"] - final_state["pos"]) < 0.1:
                return
            
    def lose_control(supervisor):
        robot_node = supervisor.getFromDef(robot_name)
        #set the new translation
        translation_field = robot_node.getField("translation")
        translation_field.setSFVec3f([200,200,0])
        m1_motor.setVelocity(0)
        m2_motor.setVelocity(0)
        m3_motor.setVelocity(0)
        m4_motor.setVelocity(0)
    
    robot_name = robot.getName()
    print(f"Robot name: {robot_name}")    
    robot_num = int(robot_name[-1])
    random.seed(SEED*robot_num)

    supervisor = Supervisor()
     # Get the root node
    root = supervisor.getRoot()

    # Get the children field of the root node
    children_field = root.getField("children")
    num_children = children_field.getCount()

    # List to hold obstacle names
    obstacle_names = []

    # Iterate through children nodes
    for i in range(num_children):
        child_node = children_field.getMFNode(i)

        # Check if the node is of type 'Robot'
        if child_node is not None and child_node.getTypeName() == "OilBarrel":
            # Access the 'name' field of the robot node
            name_field = child_node.getField("name")
            if name_field:
                obstacle_name = name_field.getSFString()  # Use getSFString() to get the single string value
                obstacle_names.append(obstacle_name)  # Append the robot name to the list

    # Output the obstacle names
    print("Obstacle names in the environment:")
    for name in obstacle_names:
        print(name)

    # get objects
    print(f"obstacle names: {obstacle_names}")
    obstacle_objects = [supervisor.getFromDef(name) for name in obstacle_names]

    # get locations and radii
    obstacle_locations = [obstacle.getField("translation").getSFVec3f() for obstacle in obstacle_objects]
    obstacle_radii = [obstacle.getField("radius").getSFFloat() for obstacle in obstacle_objects]

    find_location_flag = False
    while not find_location_flag:
        translation_new  = [(random.random()-0.5)*10.0, (random.random()-0.5)*10.0, 1 + random.random()*2.0]
        if all([np.linalg.norm(np.array(translation_new[:2]) - np.array([obstacle[0], obstacle[1]]) )\
                 > radius + 0.5 for obstacle, radius in zip(obstacle_locations, obstacle_radii)]):
            find_location_flag = True


    radius = 1 + random.random()*4.0
    r = random.random()
    thetha = random.random()*2.0*np.pi
    altitude_goal = -1

    # set the channel to communicate with the cpu
    drone_channel = ENEMY_DRONE_CHANNEL
    receiver.setChannel(drone_channel)
    # wait for the cpu to send the positions
    drone_robot = supervisor.getFromDef("Drone")
    robot_node = supervisor.getFromDef(robot_name)
    #set the new translation
    translation_field = robot_node.getField("translation")
    translation_field.setSFVec3f(translation_new[:2]+[0])
    # get the drone robot
    drone_robot = supervisor.getFromDef("Drone")


    altitude_goal = translation_new[2]
    # lifting off
    while robot.step(timestep) != -1 and gps.getValues()[2] is None:
        pass

    adverserial = ADVERSARIAL
    inplace = INPLACE
    moving = MOVING


    current_location = gps.getValues()
    x_goal = current_location[0]
    y_goal = current_location[1]
    height_desired = current_location[2]
    go_to_goal(x_goal, y_goal, altitude_goal, adverserial)
    # print('finished lifting off')
    plan_coords = None
    
    first_time = 0
    TIME_TO_SEND = 0.5
    time_to_stay = random.poisson(lam=LAMBDA)
    while robot.step(timestep) != -1:
        current_location = gps.getValues()
        current_time = robot.getTime()
        # Check if Drone is about to crash into you
        drone_position = get_enemy_drones_positions(["Drone"], [drone_robot])["Drone"]
        distance_from_drone = np.linalg.norm(np.array(drone_position) - np.array(current_location))
        if distance_from_drone < EPSILON:
            print(f"Drone is about to crash into {robot_name}")
            # enemy drone loses control
            lose_control(supervisor=supervisor)
            break
        elif moving and current_time - first_time > time_to_stay:
            first_time = current_time
            time_to_stay = random.poisson(lam=LAMBDA)
            # print('moving to new location')
            find_location_flag = False
            while not find_location_flag:
                translation_new  = [(random.random()-0.5)*10.0, (random.random()-0.5)*10.0, 1 + random.random()*3.0]
                if not OBSTACLES and np.min(np.abs(np.array(translation_new) - np.array(current_location))) > 0.3:
                    find_location_flag = True
                elif all([np.linalg.norm(np.array(translation_new[:2]) - np.array([obstacle[0], obstacle[1]]) )\
                        > radius + 0.5 for obstacle, radius in zip(obstacle_locations, obstacle_radii)]):
                    find_location_flag = True
            go_to_goal(translation_new[0], translation_new[1], translation_new[2], adverserial)
        elif inplace:
            stay_in_position(robot_name, adverserial)
        else:
            stay_in_position(robot_name, adverserial)
            distance = np.linalg.norm(np.array([x_goal, y_goal, altitude_goal]) - np.array(current_location))
            if distance < 0.1:
                find_location_flag = False
                while not find_location_flag:
                    alpha = random.random()*2.0*np.pi
                    if r < 0.3:
                        r = r + 0.2 + (radius - (r + 0.2))*random.random()
                    else:
                        r = radius*random.random()
                    prev_thetha = thetha
                    thetha = prev_thetha + np.pi/2.0 + random.random()*np.pi
                    del_x, del_y, del_z = r*np.cos(alpha)*np.sin(thetha), r*np.sin(alpha)*np.sin(thetha), r*np.cos(thetha)
                    translation_new = [float(x_goal+ del_x),  float(y_goal + del_y),  float(np.clip(altitude_goal + del_z, 1, 8))]
                    if not OBSTACLES and np.min(np.abs(np.array(translation_new) - np.array(current_location))) > 0.5:
                        find_location_flag = True
                    elif all([np.linalg.norm(np.array(translation_new[:2]) - np.array([obstacle[0], obstacle[1]]) )\
                            > radius + 0.5 for obstacle, radius in zip(obstacle_locations, obstacle_radii)]) and np.linalg.norm(np.array(translation_new[:2]) - np.array([x_global,y_global])) > 0.3:
                        find_location_flag = True
                x_goal, y_goal, altitude_goal =  translation_new
                go_to_goal(x_goal, y_goal, altitude_goal, adverserial)



if __name__ == '__main__':
    robot = Robot()
    run_robot(robot)
    