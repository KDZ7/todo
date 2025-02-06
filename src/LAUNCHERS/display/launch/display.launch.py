# --------------------------------------------------------------------------------
# GROUP: LAUNCHERS
# PACKAGE: display
# FILE: relay.launch.py
# DESCRIPTION: Launches RViz and/or Gazebo based on YAML configuration.
# AUTHOR: KDZ7
# DATE: 01/02/2025
# --------------------------------------------------------------------------------

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from relaymd import Relay

class Opaque:
    def __init__(self, function):
        self.function = function
    def __call__(self, *args, **kwargs):
        return OpaqueFunction(function=self.function, args=args, kwargs=kwargs)


def read_config(input):
    if os.path.isfile(input):
        with open(input, "r", encoding="utf-8") as file:
            return yaml.safe_load(file)
    return yaml.safe_load(input)

def generate_launch_description():

    config = LaunchConfiguration("config")
    config_data = read_config(config.perform())
    if "pkg_robot_description" in config_data:
        robot_description = Command("xacro --inorder " + PathJoinSubstitution([FindPackageShare(config_data["pkg_robot_description"]), config_data["robot_description"]]))
    else:
        robot_description = Command("xacro --inorder " + config_data["robot_description"])

    _config = DeclareLaunchArgument(
        "config",
        default_value=None,
        description=""    )

    actions = []

    set_environments = [
        SetEnvironmentVariable("ROS_DOMAIN_ID", config_data["ros_domain_id"]),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([FindPackageShare("display"), "models"])),
    ]
    actions.extend(set_environments)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=config_data["namespace"],
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": config_data["use_sim_time"]}
        ]
    )
    actions.append(robot_state_publisher)



    if config_data["display_on"] == "rviz" or config_data["display_on"] == "both":
        rviz = Node(
            package="rviz2",
            executable="rviz2",
            namespace=config_data["namespace"],
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([FindPackageShare("display"), "rviz", "config.rviz"])]
        )
        actions.append(rviz)



