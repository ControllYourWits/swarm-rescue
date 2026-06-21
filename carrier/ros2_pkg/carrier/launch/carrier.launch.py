from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    args = [
        DeclareLaunchArgument("use_nav2", default_value="false"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyS3"),
        DeclareLaunchArgument("router_ip",   default_value="192.168.8.1"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("follow_target", default_value="/scout/odom", description="Topic of the target to follow"),
        DeclareLaunchArgument("controller_mode", default_value="direct", description="Follow controller mode: direct or nav2"),
    ]
    
    use_nav2 = LaunchConfiguration("use_nav2")
    follow_target = LaunchConfiguration("follow_target")
    nav2_params_file = PathJoinSubstitution([FindPackageShare("carrier"), "config", "nav2_params.yaml"])

    nodes = [
        Node(package="carrier", executable="serial_bridge",
             parameters=[{"port": LaunchConfiguration("serial_port"), "baud": 921600}],
             output="screen"),
        
        Node(package="carrier", executable="follow_navigator",
             parameters=[{"follow_dist": 2.0, "namespace": "carrier",
                          "target_topic": follow_target,
                          "controller_mode": LaunchConfiguration("controller_mode")}],
             output="screen"),
             
        TimerAction(period=2.0, actions=[
            Node(package="carrier", executable="relay_manager",
                 parameters=[{"router_ip": LaunchConfiguration("router_ip")}]),
            Node(package="carrier", executable="supply_manager"),
        ]),
        
        # LoRa bridge
        TimerAction(period=4.0, actions=[
            Node(package="carrier", executable="lora_bridge",
                 parameters=[{"port": "/dev/ttyUSB2", "robot_id": "carrier",
                              "slot_offset": 0.33}],
                 output="screen"),
        ]),

        # Nav2 integration
        TimerAction(
            period=4.0,
            actions=[
                GroupAction(
                    condition=IfCondition(use_nav2),
                    actions=[
                        Node(
                            package="rplidar_ros",
                            executable="rplidar_composition",
                            name="carrier_rplidar",
                            parameters=[
                                {
                                    "serial_port": LaunchConfiguration("lidar_port"),
                                    "serial_baudrate": 115200,
                                    "frame_id": "carrier/laser_link",
                                    "angle_compensate": True,
                                }
                            ],
                            remappings=[("/scan", "/carrier/scan")],
                        ),
                        Node(
                            package="tf2_ros",
                            executable="static_transform_publisher",
                            arguments=["0", "0", "0.12", "0", "0", "0", "1", "carrier/base_link", "carrier/laser_link"],
                        ),
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
