from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    args = [
        DeclareLaunchArgument("use_nav2",       default_value="false"),
        DeclareLaunchArgument("serial_port",    default_value="/dev/ttyS3"),
        DeclareLaunchArgument("thermal_sim",    default_value="true"),
        DeclareLaunchArgument("lidar_port",     default_value="/dev/ttyUSB0"),
    ]
    
    use_nav2 = LaunchConfiguration("use_nav2")
    nav2_params_file = PathJoinSubstitution([FindPackageShare("specialist"), "config", "nav2_params.yaml"])

    nodes = [
        Node(package="specialist", executable="serial_bridge",
             parameters=[{"port": LaunchConfiguration("serial_port")}],
             output="screen"),
        Node(package="specialist", executable="arm_planner", output="screen"),
        
        # Share follow_navigator from carrier package for following Scout
        Node(package="carrier", executable="follow_navigator",
             name="specialist_follow_nav",
             parameters=[{"follow_dist": 3.5, "namespace": "specialist"}], output="screen"),

        TimerAction(period=2.0, actions=[
            Node(package="specialist", executable="thermal_node",
                 parameters=[{"use_sim": LaunchConfiguration("thermal_sim")}],
                 output="screen"),
            Node(package="rplidar_ros", executable="rplidar_composition",
                 name="spec_rplidar",
                 parameters=[{"serial_port": LaunchConfiguration("lidar_port"),
                               "frame_id": "specialist/laser_link"}],
                 remappings=[("/scan", "/specialist/scan")]),
        ]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["0","0","0.10","0","0","0","1",
                        "specialist/base_link","specialist/laser_link"]),

        # Nav2 integration
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
                        )
                    ]
                )
            ]
        ),
    ]
    return LaunchDescription(args + nodes)
