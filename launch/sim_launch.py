#!/usr/bin/env python

import os
import launch
from launch import LaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

PACKAGE_NAME = "webots_ros2_omni"

def load_urdf(path):
    with open(path, 'r') as file:
        return file.read()

def get_ros2_nodes(*args):
    package_dir = get_package_share_directory(PACKAGE_NAME)

    # UR3 robot setup
    ur3_control_params = os.path.join(package_dir, "resource", "ur3_control.yaml")
    ur3_driver = WebotsController(
        robot_name="Ur3Robot",
        parameters=[
            {"robot_description": load_urdf(os.path.join(package_dir, "resource", "ur3_control.urdf"))},
            {"use_sim_time": True},
            {"set_robot_state_publisher": False},
            {"update_rate": 30},
            ur3_control_params,
        ],
        respawn=True,
        namespace="ur3",
    )

    # Omni robot setup
    omni_control_params = os.path.join(package_dir, "resource", "omni_control.yaml")
    omni_driver = WebotsController(
        robot_name="PhantomOmni",
        parameters=[
            {"robot_description": load_urdf(os.path.join(package_dir, "resource", "omni.urdf"))},
            {"use_sim_time": True},
            {"set_robot_state_publisher": False},
            {"update_rate": 30},
            omni_control_params,
        ],
        respawn=True,
        namespace="omni",
    )

    controller_manager_timeout = ['--controller-manager-timeout', '500']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    # ROS2 control spawners for UR3
    ur3_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        namespace="ur3",
        arguments=["ur_joint_trajectory_controller", "-c", "/ur3/controller_manager"] + controller_manager_timeout,
    )
    ur3_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        namespace="ur3",
        arguments=["ur_joint_state_broadcaster", "-c", "/ur3/controller_manager"] + controller_manager_timeout,
    )

    # ROS2 control spawners for Omni
    omni_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        namespace="omni",
        arguments=["omni_joint_trajectory_controller", "-c", "/omni/controller_manager"] + controller_manager_timeout,
    )
    omni_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        namespace="omni",
        arguments=["omni_joint_state_broadcaster", "-c", "/omni/controller_manager"] + controller_manager_timeout,
    )

    # Waiting nodes for controller connection
    ur3_control_spawners = [ur3_trajectory_controller_spawner, ur3_joint_state_broadcaster_spawner]
    omni_control_spawners = [omni_trajectory_controller_spawner, omni_joint_state_broadcaster_spawner]

    ur3_wait_for_connection = WaitForControllerConnection(
        target_driver=ur3_driver, nodes_to_start=ur3_control_spawners
    )
    omni_wait_for_connection = WaitForControllerConnection(
        target_driver=omni_driver, nodes_to_start=omni_control_spawners
    )

    return [omni_driver, ur3_driver, omni_wait_for_connection, ur3_wait_for_connection]

def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)

    # Webots launcher
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", "ur_omni.wbt"])
    )
    ros2_supervisor = Ros2SupervisorLauncher()

    # Robot state publishers
    ur3_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        namespace="ur3",
        parameters=[
            {"robot_description": load_urdf(os.path.join(package_dir, "resource", "ur3_robot.urdf"))},
            {"use_sim_time": True},
        ],
    )

    omni_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        namespace="omni",
        parameters=[
            {"robot_description": load_urdf(os.path.join(package_dir, "resource", "omni.urdf"))},
            {"use_sim_time": True},
        ],
    )

    # Event handlers for simulation reset
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=ros2_supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    webots_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    return LaunchDescription(
        [
            webots,
            ros2_supervisor,
            ur3_state_publisher,
            omni_state_publisher,
            webots_event_handler,
            reset_handler,
        ] + get_ros2_nodes()
    )
