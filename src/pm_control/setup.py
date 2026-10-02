from setuptools import find_packages, setup
from glob import glob

package_name = "pm_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(
        include=["pm_control",
                 "pm_control.*"]
    ),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/msg", glob("msg/*.msg"))
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
            "vehicle_interface_node = pm_control.vehicle_interface_node:main",
            "wheel_odometry_node = pm_control.wheel_odometry_node:main",
            'dummy_jointstate_pub = pm_control.tools.dummy_jointstate_pub:main',
            "joint_state_bridge_node = pm_control.joint_state_bridge_node:main",
        ],
    },
)
