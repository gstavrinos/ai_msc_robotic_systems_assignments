#!/usr/bin/env python3
import os
import math
import random 
from launch import LaunchDescription 
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_pal.substitutions import LoadFile
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    gui = LaunchConfiguration("gui")

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='false',
        description='Specify if gzclient should run'
    )

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

    rosbag_data = LaunchConfiguration("rosbag_data")

    rosbag_data_arg = DeclareLaunchArgument(
        "rosbag_data",
        default_value="false"
    )

    target_node_name = LaunchConfiguration("target_node_name")

    target_node_name_arg = DeclareLaunchArgument(
        "target_node_name",
        default_value="placeholder_node_name"
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("tt_umpire"), "/launch", "/tiago_gazebo.launch.py"]
        ),
        launch_arguments={"gui": gui, "moveit": PythonExpression(["'", assignment, "' == '3'"]), "navigation":PythonExpression(["'", assignment, "' >= '1'"])}.items(),
        condition=UnlessCondition(rosbag_data),
    )

    table_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_table_tennis"), "/launch", "/table_description.launch.py"]
        ),
        launch_arguments={"easy_collision":PythonExpression(["'", easy_mode, "'"])}.items(),
        condition=UnlessCondition(rosbag_data),
    )

    racket_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_table_tennis"), "/launch", "/racket_description.launch.py"]
        ),
        condition=UnlessCondition(rosbag_data),
    )

    ball_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_table_tennis"), "/launch", "/ball_description.launch.py"]
        ),
        condition=UnlessCondition(rosbag_data),
    )

    spawn_table = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_table",
            arguments=["-entity", "table", "-topic", "table_description", "-x", "2.64", "-y", "0", "-z", "0"],
            output="screen",
            condition=UnlessCondition(rosbag_data),
            )

    table_tf = Node(
            package="odom_to_tf_ros2",
            executable="odom_to_tf",
            name="table_odom_to_tf",
            condition=UnlessCondition(rosbag_data),
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
            condition=UnlessCondition(rosbag_data),
            )

    racket_tf = Node(
            package="odom_to_tf_ros2",
            executable="odom_to_tf",
            name="racket_odom_to_tf",
            condition=UnlessCondition(rosbag_data),
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
            condition=UnlessCondition(rosbag_data),
            )

    ball_tf = Node(
            condition=IfCondition(PythonExpression(
                ["'", assignment, "' == '0' or '", assignment, "' > '1' and not '", rosbag_data, "'"]
                )),
            package="odom_to_tf_ros2",
            executable="odom_to_tf",
            name="ball_odom_to_tf",
            parameters=[
                {"use_sim_time": True,"odom_topic": "/p3d/ball_odom"},
                ]
            )

    rosbag_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", os.path.normpath(os.path.abspath(__file__)+"/../../../../../../build/tt_umpire/_deps/rosbag-src/"),"--loop", "--clock"],
        output="screen",
            condition=IfCondition(rosbag_data),
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
                {"assignment": LaunchConfiguration("assignment"),"target_node_name": target_node_name},
                ]
            )

    basic_navigator_sim = Node(
            package="tt_umpire",
            executable="basic_navigator_sim.py",
            name="basic_navigator_sim",
            condition=IfCondition(rosbag_data),
            )

    target_node_configure = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", target_node_name, "configure"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", assignment, "' > '0'"])),
    )

    target_node_activate = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", target_node_name, "activate"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", assignment, "' > '0'"])),
    )

    load_map_with_table = ExecuteProcess(
            cmd=["ros2", "service", "call", "/map_server/load_map", "nav2_msgs/srv/LoadMap", "map_url: '"+os.path.join(
            get_package_share_directory("tt_umpire"),
            "maps", "pal_with_tt_table.yaml")+"'"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", assignment, "' == '2' and '", easy_mode,"'"])),
    )


    return LaunchDescription(
        [
        gui_arg,
        rosbag_data_arg,
        assignment_arg,
        easy_mode_arg,
        target_node_name_arg,

        rosbag_play,
        basic_navigator_sim,

        gazebo_launch,

        TimerAction(
            period = 10.0,
            actions = [
                GroupAction(
                    actions=[
                        table_description_launch,
                        spawn_table,
                        table_tf,
                        load_map_with_table,
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
                target_action=spawn_ball,
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
                target_action=rosbag_play,
                on_start=[
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

