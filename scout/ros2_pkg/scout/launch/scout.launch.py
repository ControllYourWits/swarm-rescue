from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    args = [
        DeclareLaunchArgument("use_nav2", default_value="false", description="Use Nav2 instead of RL policy"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyS3"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("radar_port", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument("model_path", default_value="/home/robot/models/scout_policy.onnx"),
        DeclareLaunchArgument("norm_path", default_value="/home/robot/models/scout_norm.npz"),
    ]
    
    use_nav2 = LaunchConfiguration("use_nav2")
    
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare("scout"), "config", "nav2_params.yaml"]
    )

    nodes = [
        Node(
            package="scout",
            executable="serial_bridge",
            parameters=[{"port": LaunchConfiguration("serial_port"), "baud": 921600, "cmd_timeout": 0.5}],
            output="screen",
        ),
        Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            name="rplidar",
            parameters=[
                {
                    "serial_port": LaunchConfiguration("lidar_port"),
                    "serial_baudrate": 115200,
                    "frame_id": "scout/laser_link",
                    "angle_compensate": True,
                }
            ],
            remappings=[("/scan", "/scout/scan")],
        ),
        Node(package="scout", executable="lidar_proc", output="screen"),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="slam_toolbox",
                    executable="async_slam_toolbox_node",
                    name="scout_slam",
                    parameters=[
                        {
                            "odom_frame": "scout/odom",
                            "map_frame": "scout/map",
                            "base_frame": "scout/base_link",
                            "scan_topic": "/scout/scan",
                            "use_sim_time": False,
                            "resolution": 0.05,
                            "mode": "mapping",
                        }
                    ],
                )
            ],
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="scout",
                    executable="radar_processor",
                    parameters=[{"port": LaunchConfiguration("radar_port")}],
                    output="screen",
                )
            ],
        ),
        TimerAction(period=3.0, actions=[Node(package="scout", executable="life_map_node", output="screen")]),
        
        # RL Navigation (Only if use_nav2 is false)
        TimerAction(
            period=3.0,
            actions=[
                GroupAction(
                    condition=UnlessCondition(use_nav2),
                    actions=[
                        Node(
                            package="scout",
                            executable="rl_nav_node",
                            parameters=[
                                {
                                    "model_path": LaunchConfiguration("model_path"),
                                    "norm_path": LaunchConfiguration("norm_path"),
                                    "max_vx": 0.6,
                                    "safety_dist": 0.30,
                                }
                            ],
                            output="screen",
                        )
                    ]
                )
            ],
        ),
        
        # Nav2 Navigation (Only if use_nav2 is true)
        TimerAction(
            period=4.0,
            actions=[
                GroupAction(
                    condition=IfCondition(use_nav2),
                    actions=[
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])
                            ),
                            launch_arguments={
                                'use_sim_time': 'False',
                                'params_file': nav2_params_file,
                                'autostart': 'True',
                                'use_composition': 'False'
                            }.items()
                        ),
                        Node(
                            package="scout",
                            executable="nav2_goal_bridge",
                            output="screen",
                        )
                    ]
                )
            ]
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0.12", "0", "0", "0", "1", "scout/base_link", "scout/laser_link"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.04", "0", "0.06", "0", "0", "0", "1", "scout/base_link", "scout/imu_link"],
        ),
    ]
    return LaunchDescription(args + nodes)
