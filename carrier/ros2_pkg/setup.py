from setuptools import setup
from glob import glob
package_name = "carrier"
setup(name=package_name, version="1.0.0",
    packages=[package_name, f"{package_name}.nodes"],
    data_files=[
        ("share/ament_index/resource_index/packages",[f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("carrier/launch/*.py")),
        (f"share/{package_name}/config", glob("carrier/config/*.yaml")),
    ],
    install_requires=["setuptools","pyserial","numpy"],
    zip_safe=True,
    entry_points={"console_scripts":[
        "serial_bridge    = carrier.nodes.serial_bridge:main",
        "follow_navigator = carrier.nodes.follow_navigator:main",
        "relay_manager    = carrier.nodes.relay_manager:main",
        "supply_manager   = carrier.nodes.supply_manager:main",
        "lora_bridge      = carrier.nodes.lora_bridge_node:main",
        "human_simulator  = carrier.nodes.human_simulator:main",
    ]})
