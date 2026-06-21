from setuptools import setup, find_packages
import os

package_name = "ground_station"

# Include templates for the web dashboard
data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
]

# Add template files
for dirpath, dirnames, filenames in os.walk(os.path.join(package_name, "templates")):
    for f in filenames:
        dest = os.path.join("share", package_name, "templates")
        data_files.append((dest, [os.path.join(dirpath, f)]))

setup(
    name=package_name,
    version="1.1.0",
    packages=find_packages(),
    data_files=data_files,
    install_requires=["setuptools", "flask", "flask-socketio"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "ground_station_node = ground_station.ground_station_node:main",
            "web_dashboard = ground_station.web_dashboard:main",
            "task_coordinator = ground_station.task_coordinator:main",
        ]
    },
)
