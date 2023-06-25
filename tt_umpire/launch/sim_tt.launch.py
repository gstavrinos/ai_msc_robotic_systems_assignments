#!/usr/bin/env python3
import os
import math
import random 
from launch import LaunchDescription 
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_pal.substitutions import LoadFile
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    assignment = LaunchConfiguration("assignment")

    assignment_arg = DeclareLaunchArgument(
        "assignment",
        default_value="0"
    )

    easy_mode = LaunchConfiguration("easy_mode")

    easy_mode_arg = DeclareLaunchArgument(
        "easy_mode",
        default_value="false"
    )

    target_node_name = LaunchConfiguration("target_node_name")

    target_node_name_arg = DeclareLaunchArgument(
        "target_node_name",
        default_value="placeholder_node_name"
    )

    table_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_table_tennis"), "/launch", "/table_description.launch.py"]
        ),
        launch_arguments={"easy_collision":PythonExpression(["'", easy_mode, "'"])}.items(),
    )

    racket_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_table_tennis"), "/launch", "/racket_description.launch.py"]
        ),
    )

    ball_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_table_tennis"), "/launch", "/ball_description.launch.py"]
        ),
    )

    spawn_table = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_table",
            arguments=["-entity", "table", "-topic", "table_description", "-x", "2.64", "-y", "0", "-z", "0"],
            output="screen",
            )

    table_tf = Node(
            package="odom_to_tf_ros2",
            executable="odom_to_tf",
            name="table_odom_to_tf",
            parameters=[
                {"use_sim_time": True,"odom_topic": "/p3d/table_odom"},
                ]
            )

    spawn_racket = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_racket",
            arguments=["-entity", "racket", "-topic", "racket_description", "-x", "1.34", "-y", "0", "-z", "0.8"],
            output="screen",
            )

    racket_tf = Node(
            package="odom_to_tf_ros2",
            executable="odom_to_tf",
            name="racket_odom_to_tf",
            parameters=[
                {"use_sim_time": True,"odom_topic": "/p3d/racket_odom"},
                ]
            )

    spawn_ball = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_ball",
            arguments=["-entity", "ball", "-topic", "ball_description", "-x", "1.4", "-y", str(random.choice(list(set(range(-35, 35)) - set(range(-9,9))))*0.01), "-z", "1.06"],
            output="screen",
            )

    ball_tf = Node(
            condition=IfCondition(PythonExpression(
                ["'", assignment, "' == '0' or '", assignment, "' > '1'"]
                )),
            package="odom_to_tf_ros2",
            executable="odom_to_tf",
            name="ball_odom_to_tf",
            parameters=[
                {"use_sim_time": True,"odom_topic": "/p3d/ball_odom"},
                ]
            )

    ball_locator_launch = ExecuteProcess(
        cmd=["ros2", "launch", "tt_ball_locator", "ball_locator.launch.py"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", assignment, "' == '1'"])),
    )

    table_explorer_launch = ExecuteProcess(
        cmd=["ros2", "launch", "tt_table_explorer", "table_explorer.launch.py"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", assignment, "' == '2'"])),
    )

    racket_handling_launch = ExecuteProcess(
        cmd=["ros2", "launch", "tt_racket_handling", "racket_handling.launch.py"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", assignment, "' == '3'"])),
    )

    tt_umpire_node = Node(
            condition=IfCondition(PythonExpression(
                ["'", assignment, "' > '0'"]
                )),
            package="tt_umpire",
            executable="tt_umpire",
            name="tt_umpire",
            parameters=[
                {"assignment": LaunchConfiguration("assignment"),"target_node_name": LaunchConfiguration("target_node_name")},
                ]
            )

    target_node_configure = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", LaunchConfiguration("target_node_name"), "configure"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", assignment, "' > '0'"])),
    )

    target_node_activate = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", LaunchConfiguration("target_node_name"), "activate"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", assignment, "' > '0'"])),
    )


    return LaunchDescription(
        [
        assignment_arg,
        easy_mode_arg,
        target_node_name_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("tiago_gazebo"), "/launch", "/tiago_gazebo.launch.py"]
            ),
            launch_arguments={"use_rviz":"true","moveit":"true", "navigation":"true"}.items()
        ),

        TimerAction(
            period = 10.0,
            actions = [
                GroupAction(
                    actions=[
                        table_description_launch,
                        spawn_table,
                        table_tf,
                        ]
                    ),
                ]
            ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_table,
                on_exit=[
                    TimerAction(
                        period = 10.0,
                        actions = [
                            GroupAction(
                                actions=[racket_description_launch, spawn_racket, racket_tf]
                            ),
                        ]
                    )
                ],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_racket,
                on_exit=[
                    TimerAction(
                        period = 2.0,
                        actions = [
                            GroupAction(
                                actions=[ball_description_launch, spawn_ball, ball_tf]
                            ),
                        ]
                    )
                ],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_table,
                on_exit=[
                    TimerAction(
                        period = 5.0,
                        actions = [
                            GroupAction(
                                actions=[ball_locator_launch, table_explorer_launch, racket_handling_launch, tt_umpire_node]
                            ),
                        ]
                    )
                ],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=tt_umpire_node,
                on_start=[
                    TimerAction(
                        period = 10.0,
                        actions = [
                            GroupAction(
                                actions=[target_node_configure]
                            ),
                        ]
                    )
                ],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=target_node_configure,
                on_exit=[
                    TimerAction(
                        period = 5.0,
                        actions = [
                            GroupAction(
                                actions=[target_node_activate]
                            ),
                        ]
                    )
                ],
            )
        ),

        ],
    )

