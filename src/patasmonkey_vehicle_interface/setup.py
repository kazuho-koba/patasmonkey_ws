from setuptools import find_packages, setup
from glob import glob

package_name = "patasmonkey_vehicle_interface"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(
        include=["patasmonkey_vehicle_interface", "patasmonkey_vehicle_interface.*"]
    ),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mujin",
    maintainer_email="kazuho.kobayashi.ynu@gmail.com",
    description="package to interface the Patasmonkey UGV",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vehicle_interface_node = patasmonkey_vehicle_interface.vehicle_interface_node:main",
            "wheel_odometry_node = patasmonkey_vehicle_interface.wheel_odometry_node:main",
            'dummy_jointstate_pub = patasmonkey_vehicle_interface.tools.dummy_jointstate_pub:main',
            "joint_state_bridge_node = patasmonkey_vehicle_interface.joint_state_bridge_node:main",
        ],
    },
)
