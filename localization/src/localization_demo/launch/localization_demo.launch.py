"""
launch/localization_demo.launch.py

Starts all four nodes in this package simultaneously.
Run with:
  ros2 launch localization_demo localization_demo.launch.py

Optional argument:
  odom_topic:=/turtlebot4/odom   (default: /odom)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Declare the odom topic argument ──────────────────────────────────────
    # Change this to /turtlebot4/odom when running on the real robot.
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic to subscribe to'
    )

    # ── Node 1: odometry_reader ───────────────────────────────────────────────
    odometry_reader_node = Node(
        package='localization_demo',
        executable='odometry_reader',
        name='odometry_reader',
        output='screen',
        remappings=[
            ('/odom', LaunchConfiguration('odom_topic'))
        ],
        parameters=[{
            'use_sim_time': False
        }]
    )

    # ── Node 2: dead_reckoning ────────────────────────────────────────────────
    dead_reckoning_node = Node(
        package='localization_demo',
        executable='dead_reckoning',
        name='dead_reckoning',
        output='screen',
        remappings=[
            ('/odom', LaunchConfiguration('odom_topic'))
        ],
        parameters=[{
            'use_sim_time': False
        }]
    )

    # ── Node 3: localization_belief ───────────────────────────────────────────
    localization_belief_node = Node(
        package='localization_demo',
        executable='localization_belief',
        name='localization_belief',
        output='screen',
        remappings=[
            ('/odom', LaunchConfiguration('odom_topic'))
        ],
        parameters=[{
            'use_sim_time': False
        }]
    )

    # ── Node 4: trajectory_logger ─────────────────────────────────────────────
    trajectory_logger_node = Node(
        package='localization_demo',
        executable='trajectory_logger',
        name='trajectory_logger',
        output='screen',
        remappings=[
            ('/odom', LaunchConfiguration('odom_topic'))
        ],
        parameters=[{
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        odom_topic_arg,
        LogInfo(msg="Starting BLM4830 Localization Demo..."),
        LogInfo(msg="Subscribing to odom topic: /odom (change with odom_topic:=...)"),
        odometry_reader_node,
        dead_reckoning_node,
        localization_belief_node,
        trajectory_logger_node,
    ])
