"""
Gazebo launch file for the three-robot rescue simulation.

The visual world is handled by Gazebo. A lightweight sim_swarm_node publishes
odom, laser sectors, goals, life detections, and swarm status so the existing
Scout, Carrier, Specialist, and dashboard nodes can be exercised together.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gz_pkg = get_package_share_directory("gazebo_ros")
    bringup = get_package_share_directory("swarm_bringup")

    world = os.path.join(bringup, "worlds", "disaster_rubble.world")
    xacro_file = os.path.join(bringup, "urdf", "rescue_robot.urdf.xacro")

    carrier_mode = LaunchConfiguration('carrier_mode')
    
    args = [
        DeclareLaunchArgument('carrier_mode', default_value='scout', description="Carrier follow mode: 'scout' or 'human'")
    ]

    robots = [
        {"name": "scout", "x": "0.0", "y": "0.0", "color": "0.90 0.20 0.18 1"},
        {"name": "carrier", "x": "-1.8", "y": "0.0", "color": "0.15 0.75 0.30 1"},
        {"name": "specialist", "x": "-3.2", "y": "-0.2", "color": "0.20 0.45 0.95 1"},
    ]

    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_pkg, "launch", "gazebo.launch.py")),
            launch_arguments={"world": world, "verbose": "false"}.items(),
        ),
        Node(package="swarm_bringup", executable="sim_swarm_node", output="screen"),
    ]

    for robot in robots:
        robot_desc = ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    xacro_file,
                    " robot_name:=",
                    robot["name"],
                    " robot_color:=",
                    robot["color"],
                ]
            ),
            value_type=str,
        )

        nodes.extend(
            [
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    namespace=robot["name"],
                    parameters=[
                        {
                            "robot_description": robot_desc,
                            "publish_frequency": 30.0,
                            "use_sim_time": True,
                        }
                    ],
                ),
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    name=f"spawn_{robot['name']}",
                    arguments=[
                        "-topic",
                        f"/{robot['name']}/robot_description",
                        "-entity",
                        robot["name"],
                        "-robot_namespace",
                        robot["name"],
                        "-x",
                        robot["x"],
                        "-y",
                        robot["y"],
                        "-z",
                        "0.12",
                    ],
                    output="screen",
                ),
            ]
        )

    is_human_mode = PythonExpression(["'", carrier_mode, "' == 'human'"])
    is_scout_mode = PythonExpression(["'", carrier_mode, "' == 'scout'"])

    nodes.append(
        TimerAction(
            period=4.0,
            actions=[
                Node(package="scout", executable="lidar_proc",
                     parameters=[{"use_sim": True}], output="screen"),
                Node(package="scout", executable="life_map_node",
                     parameters=[{"use_sim": True}], output="screen"),
                # 地形分析 (2D LiDAR 降级模式)
                Node(package="scout", executable="terrain_analysis",
                     parameters=[{"use_3d_lidar": False}], output="screen"),
                # 废墟恢复行为
                Node(package="scout", executable="disaster_recovery",
                     output="screen"),
                Node(
                    package="scout",
                    executable="rl_nav_node",
                    parameters=[{"use_sim_time": True, "max_vx": 0.45}],
                    output="screen",
                ),
                
                # Human Simulator
                GroupAction(
                    condition=IfCondition(is_human_mode),
                    actions=[
                        Node(package="carrier", executable="human_simulator", output="screen"),
                        Node(
                            package="carrier",
                            executable="follow_navigator",
                            parameters=[{"use_sim_time": True, "follow_dist": 2.0,
                                         "target_topic": "/human/odom",
                                         "namespace": "carrier",
                                         "controller_mode": "direct"}],
                            output="screen",
                        )
                    ]
                ),
                
                GroupAction(
                    condition=IfCondition(is_scout_mode),
                    actions=[
                        Node(
                            package="carrier",
                            executable="follow_navigator",
                            parameters=[{"use_sim_time": True, "follow_dist": 2.0,
                                         "target_topic": "/scout/odom",
                                         "namespace": "carrier",
                                         "controller_mode": "direct"}],
                            output="screen",
                        )
                    ]
                ),
                
                Node(package="carrier", executable="supply_manager", output="screen"),
                Node(package="specialist", executable="arm_planner", output="screen"),
                Node(
                    package="specialist",
                    executable="thermal_node",
                    parameters=[{"use_sim_time": True, "use_sim": True}],
                    output="screen",
                ),
            ],
        )
    )

    return LaunchDescription(args + nodes)
