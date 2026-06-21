"""
swarm.launch.py — 三机协同完整启动文件

用法:
  # 完整启动
  ros2 launch swarm_bringup swarm.launch.py

  # 仅启动 Scout
  ros2 launch swarm_bringup swarm.launch.py enable_carrier:=false enable_specialist:=false

  # 仿真模式
  ros2 launch swarm_bringup swarm.launch.py use_sim:=true
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                             IncludeLaunchDescription, TimerAction, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    scout_pkg      = get_package_share_directory("scout")
    carrier_pkg    = get_package_share_directory("carrier")
    specialist_pkg = get_package_share_directory("specialist")
    bringup_pkg    = get_package_share_directory("swarm_bringup")

    args = [
        DeclareLaunchArgument("enable_scout",      default_value="true"),
        DeclareLaunchArgument("enable_carrier",    default_value="true"),
        DeclareLaunchArgument("enable_specialist", default_value="true"),
        DeclareLaunchArgument("use_sim",           default_value="false"),
        DeclareLaunchArgument("scout_port",        default_value="/dev/ttyS3"),
        DeclareLaunchArgument("carrier_port",      default_value="/dev/ttyS4"),
        DeclareLaunchArgument("specialist_port",   default_value="/dev/ttyS5"),
    ]

    nodes = [
        # ── Scout ──────────────────────────────────────
        GroupAction(
            condition=IfCondition(LaunchConfiguration("enable_scout")),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(scout_pkg, "launch", "scout.launch.py")),
                launch_arguments={
                    "serial_port": LaunchConfiguration("scout_port")}.items()
            )]
        ),
        # ── Carrier ────────────────────────────────────
        TimerAction(period=3.0, actions=[
            GroupAction(
                condition=IfCondition(LaunchConfiguration("enable_carrier")),
                actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(carrier_pkg, "launch", "carrier.launch.py")),
                    launch_arguments={
                        "serial_port": LaunchConfiguration("carrier_port")}.items()
                )]
            )
        ]),
        # ── Specialist ─────────────────────────────────
        TimerAction(period=3.0, actions=[
            GroupAction(
                condition=IfCondition(LaunchConfiguration("enable_specialist")),
                actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(specialist_pkg,"launch","specialist.launch.py")),
                    launch_arguments={
                        "serial_port": LaunchConfiguration("specialist_port")}.items()
                )]
            )
        ]),
        # ── 多机地图合并 ────────────────────────────────
        TimerAction(period=8.0, actions=[
            Node(package="multirobot_map_merge",
                 executable="map_merge",
                 name="swarm_map_merge",
                 parameters=[os.path.join(
                     bringup_pkg, "config", "map_merge.yaml")])
        ]),
        # ── 地面站 ─────────────────────────────────────
        TimerAction(period=5.0, actions=[
            Node(package="ground_station",
                 executable="ground_station_node",
                 output="screen")
        ]),
        # ── RViz2 可视化 ───────────────────────────────
        TimerAction(period=6.0, actions=[
            Node(package="rviz2", executable="rviz2",
                 arguments=["-d", os.path.join(
                     get_package_share_directory("swarm_bringup"),
                     "config", "swarm.rviz")],
                 output="screen")
        ]),
        LogInfo(msg="[swarm_bringup] All robots launched. "
                    "Use ground station or /scout/arm_task etc."),
    ]
    return LaunchDescription(args + nodes)
