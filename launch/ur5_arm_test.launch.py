# pylint: disable=missing-function-docstring, trailing-whitespace, missing-module-docstring, line-too-long, unspecified-encoding, unused-import, missing-final-newline

# from typing import Callable

# imports used
from __future__ import annotations
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # get the file path of my_moveit_pkg
    package_share_path = get_package_share_directory('my_moveit_pkg')
    
    # files taken as parameters by nodes
    robot_description = os.path.join(package_share_path, 'config', 'ur5.urdf')
    robot_description_semantic = os.path.join(package_share_path, 'config', 'ur5.srdf')
    robot_description_kinematics = os.path.join(package_share_path, 'config', 'kinematics.yaml')
    # planning_pipelines = os.path.join(package_share_path, "config", "ompl_planning.yaml")

    # open and read the robot_description (ur5.urdf)
    with open(robot_description, 'r') as f:
        robot_descript = f.read()

    # open and read the robot_descripiton_semantic (ur5.srdf)
    with open(robot_description_semantic, 'r') as f:
        robot_descript_semantic = f.read()

    # # moveit argument for ur5_arm node
    # moveit_config = (MoveItConfigsBuilder(robot_name = "ur")
    #     .robot_description(file_path = package_share_path + "/config/ur5.urdf")
    #     .robot_description_semantic(file_path = package_share_path + "/config/ur5.srdf")
    #     .trajectory_execution(file_path = package_share_path + "/config/controllers.yaml")
    #     .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
    #     .planning_pipelines(pipelines = ["ompl"])
    #     .to_moveit_configs()
    # )

    # moveit_config.planning_pipelines["ompl"]["ur_manipulator"]["projection_evaluator"] = "joints(shoulder_pan_joint,shoulder_lift_joint)"
    
    # node that runs the ur5_arm_test.py python script
    ur5_arm = Node(
                name = "ur5_arm_test",
                package = "my_moveit_pkg",
                executable = "ur5_arm_test",
                output = "screen",
                parameters = [robot_description_kinematics, 
                            #   planning_pipelines,
                              {'robot_description': robot_descript, 
                               'robot_description_semantic': robot_descript_semantic, 
                               'use_sim_time': True}]
    )

    return LaunchDescription(
            [ 
                # launch the ur5_arm node
                ur5_arm
        ]
    )