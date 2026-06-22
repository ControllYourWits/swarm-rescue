import os
from setuptools import setup

package_name = "swarm_bringup"
_here = os.path.abspath(os.path.dirname(__file__))


def _collect_files(directory):
    """Collect all regular files from a directory, recursively.
    Returns list of (install_subdir, [file_paths]) tuples for data_files."""
    result = []
    base = os.path.join(_here, directory)
    if not os.path.isdir(base):
        return result
    for dirpath, dirnames, filenames in os.walk(base):
        if not filenames:
            continue
        # Relative dir from package root, e.g. "urdf/carrier"
        rel_dir = os.path.relpath(dirpath, _here)
        install_dir = os.path.join("share", package_name, rel_dir)
        files = [os.path.join(rel_dir, f) for f in filenames]
        result.append((install_dir, files))
    return result


# Build data_files list
_data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
]

# Add launch files (flat directory)
_launch_dir = os.path.join(_here, "launch")
if os.path.isdir(_launch_dir):
    _launch_py = [os.path.join("launch", f) for f in os.listdir(_launch_dir) if f.endswith(".py")]
    _data_files.append((f"share/{package_name}/launch", _launch_py))

# Add config, worlds, urdf (may have subdirectories)
for subdir in ["config", "worlds", "urdf"]:
    _data_files.extend(_collect_files(subdir))

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name, f"{package_name}.shared"],
    data_files=_data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "sim_swarm_node = swarm_bringup.sim_swarm_node:main",
        "lifecycle_manager = swarm_bringup.lifecycle_manager:main",
    ]},
)
