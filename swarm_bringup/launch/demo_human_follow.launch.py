"""
Lightweight demo launch: No Gazebo, just sim_swarm_node + human following + RViz2.
"""
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('carrier_mode', default_value='human',
                              description="'scout' or 'human'"),

        # Simulated data publisher (odom, scans, goals, etc.)
        Node(package="swarm_bringup", executable="sim_swarm_node", output="screen"),

        # Human UWB tag simulator
        Node(package="carrier", executable="human_simulator", output="screen"),

        # Carrier follow navigator - tracking human
        TimerAction(period=2.0, actions=[
            Node(
                package="carrier",
                executable="follow_navigator",
                parameters=[{
                    "follow_dist": 1.5,
                    "namespace": "carrier",
                    "target_topic": "/human/odom",
                }],
                output="screen",
            ),
            Node(package="carrier", executable="supply_manager", output="screen"),
        ]),

        # Static TF for visualization
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "1", "scout/map", "map"],
        ),

        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
        ),
    ])
