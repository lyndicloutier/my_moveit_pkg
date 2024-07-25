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
    # moveit_package_share_path = get_package_share_directory('ur_moveit_config')

    # # files taken as parameters by nodes
    robot_description = os.path.join(package_share_path, 'config', 'ur5.urdf')
    robot_description_semantic = os.path.join(package_share_path, 'config', 'ur5.srdf')
    robot_description_kinematics = os.path.join(package_share_path, 'config', 'kinematics.yaml')
    # joint_limits = os.path.join(package_share_path, 'config', 'joint_limits.yaml')
    # planning_scene_monitor
    # planning_pipelines
    # pilz_cartesian_limits
    # trajectory_execution = os.path.join(moveit_package_share_path, 'config', 'controllers.yaml')
    # position_yaml = os.path.join(get_package_share_directory('my_moveit_pkg'), 'config', 'ur5_arm_test_goal_config.yaml')

    # open and read the robot_description (ur5.urdf)
    with open(robot_description, 'r') as f:
        robot_descript = f.read()

    # open and read the robot_descripiton_semantic (ur5.srdf)
    with open(robot_description_semantic, 'r') as f:
        robot_descript_semantic = f.read()

    # open and read the robot_descripiton_kinematics (kinematics.yaml)
    # with open(robot_description_kinematics, 'r') as f:
    #     robot_descript_kinematics = yaml.safe_load(f)


    # moveit argument for ur5_arm node
    # moveit_config = (MoveItConfigsBuilder(robot_name = "ur")
    #     .robot_description(file_path = package_share_path + "/config/ur5.urdf")
    #     .robot_description_semantic(file_path = package_share_path + "/config/ur5.srdf")
    #     .trajectory_execution(file_path = moveit_package_share_path + "/config/controllers.yaml")
    #     .planning_pipelines(
    #         pipelines = ["ompl", "pilz_industrial_motion_planner"],
    #         default_planning_pipeline = "ompl",
    #     )
    #     .moveit_cpp(
    #         file_path = package_share_path + "/config/motion_planning_python_api_tutorial.yaml"
    #     )
    #     .to_moveit_configs()
    # )

    # example_file = DeclareLaunchArgument(
    #     "example_file",
    #     default_value = "ur5_arm_test.py",
    #     description = "Python API tutorial file name",
    # )

    # node that runs the ur5_arm_test.py python script
    ur5_arm = Node(
                name = "ur5_arm_test",
                package = "my_moveit_pkg",
                executable = "ur5_arm_test",
                output = "screen",
                # parameters = [position_yaml, moveit_config.to_dict(), {"use_sim_time": True}],
                parameters = [robot_description_kinematics, {'robot_description': robot_descript, 
                               'robot_description_semantic': robot_descript_semantic, 

                            #    'robot_description_kinematics': robot_descript_kinematics
                               }],
                arguments = [robot_description, robot_description_semantic
                            #  , robot_description_kinematics
                             ]
    )
    
    # moveit_node = Node(
    #             name = "moveit_py",
    #             package = "moveit2_tutorials",
    #             executable = LaunchConfiguration("example_file"),
    #             output = "both",
    #             parameters = [moveit_config.to_dict()],
    # )

    return LaunchDescription(
            [ 
                # moveit_config,
                # example_file,
                # moveit_node,

                # launch the ur5_arm node
                ur5_arm
        ]
    )