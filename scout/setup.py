from setuptools import setup
from glob import glob
package_name = "scout"
setup(name=package_name, version="1.0.0",
    packages=[package_name, f"{package_name}.nodes"],
    data_files=[
        ("share/ament_index/resource_index/packages",[f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("scout/launch/*.py")),
        (f"share/{package_name}/config", glob("scout/config/*.yaml")),
    ],
    install_requires=["setuptools","pyserial","numpy","onnxruntime"],
    zip_safe=True,
    entry_points={"console_scripts":[
        "serial_bridge  = scout.nodes.serial_bridge:main",
        "lidar_proc     = scout.nodes.lidar_proc:main",
        "radar_processor= scout.nodes.radar_processor:main",
        "life_map_node  = scout.nodes.life_map_node:main",
        "rl_nav_node    = scout.nodes.rl_nav_node:main",
        "nav2_goal_bridge = scout.nodes.nav2_goal_bridge:main",
        "terrain_analysis = scout.nodes.terrain_analysis:main",
        "disaster_recovery = scout.nodes.disaster_recovery:main",
    ]})
