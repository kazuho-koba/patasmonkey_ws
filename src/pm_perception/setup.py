from glob import glob
from setuptools import find_packages, setup

package_name = "pm_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kazuho",
    maintainer_email="kazuho@example.com",
    description="Lightweight local terrain perception for Patasmonkey",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "depth_elevation_mapper_node = "
            "pm_perception.depth_elevation_mapper_node:main",
        ],
    },
)
