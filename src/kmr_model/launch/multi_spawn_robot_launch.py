#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
import math


def generate_launch_description():

    sdf = os.path.join(get_package_share_directory('kmr_model'), 'robot/', 'model.sdf')
    pkg_box_bot_description = get_package_share_directory('kmr_model')
    #assert os.path.exists(urdf), "Thebox_bot.urdf doesnt exist in "+str(urdf)

    # Names and poses of the robots

    yaw1=3.1415
    #yaw1=-1.57079633
    pitch1=0.0      #Always zero (planar robot)
    roll1=0.0      #Always zero (planar robot)

    cy = math.cos(yaw1*0.5); sy = math.sin(yaw1*0.5);  
    cp = math.cos(pitch1*0.5); sp = math.sin(pitch1*0.5)
    cr = math.cos(roll1*0.5); sr = math.sin(roll1*0.5)

    q1=[0]*4
    q1[0] = cy*cp*cr+sy*sp*sr; q1[1]=cy*cp*sr-sy*sp*cr
    q1[2] = sy*cp*sr+cy*sp*cr; q1[3]=sy*cp*cr-cy*sp*sr

    yaw2=0.0
    pitch2=0.0      #Always zero (planar robot)
    roll2=0.0      #Always zero (planar robot)

    cy = math.cos(yaw2*0.5); sy = math.sin(yaw2*0.5);  
    cp = math.cos(pitch2*0.5); sp = math.sin(pitch2*0.5)
    cr = math.cos(roll2*0.5); sr = math.sin(roll2*0.5)

    q2=[0]*4
    q2[0] = cy*cp*cr+sy*sp*sr; q2[1]=cy*cp*sr-sy*sp*cr
    q2[2] = sy*cp*sr+cy*sp*cr; q2[3]=sy*cp*cr-cy*sp*sr

    robots = [
        {'name': 'robot1', 'x_pose': 0.8, 'y_pose': -2.0, 'z_pose': 0.01, 'orien_x':q1[1], 'orien_y':q1[2], 'orien_z':q1[3], 'orien_w':q1[0]},
        {'name': 'robot2', 'x_pose': 0.8, 'y_pose': 7.0, 'z_pose': 0.01, 'orien_x':q2[1], 'orien_y':q2[2], 'orien_z':q2[3], 'orien_w':q2[0]}]


    # We create the list of spawn robots commands
    spawn_robots_cmds = []
    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_box_bot_description, 'launch',
                                                           'spawn_kuka_launch.py')),
                launch_arguments={
                                  'robot_sdf': sdf,
                                  'x': TextSubstitution(text=str(robot['x_pose'])),
                                  'y': TextSubstitution(text=str(robot['y_pose'])),
                                  'z': TextSubstitution(text=str(robot['z_pose'])),
                                  'ox': TextSubstitution(text=str(robot['orien_x'])),
                                  'oy': TextSubstitution(text=str(robot['orien_y'])),
                                  'oz': TextSubstitution(text=str(robot['orien_z'])),
                                  'ow': TextSubstitution(text=str(robot['orien_w'])),
                                  'robot_name': robot['name'],
                                  'robot_namespace': robot['name']
                                  }.items()))

    # Create the launch description and populate
    ld = LaunchDescription()
    
    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    return ld