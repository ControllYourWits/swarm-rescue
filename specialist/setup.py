from setuptools import setup
from glob import glob
package_name = "specialist"
setup(name=package_name, version="1.0.0",
    packages=[package_name, f"{package_name}.nodes"],
    data_files=[
        ("share/ament_index/resource_index/packages",[f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("specialist/launch/*.py")),
        (f"share/{package_name}/config", glob("specialist/config/*.yaml")),
    ],
    install_requires=["setuptools","pyserial","numpy","opencv-python-headless"],
    zip_safe=True,
    entry_points={"console_scripts":[
        "serial_bridge = specialist.nodes.serial_bridge:main",
        "arm_planner   = specialist.nodes.arm_planner:main",
        "thermal_node  = specialist.nodes.thermal_node:main",
    ]})
