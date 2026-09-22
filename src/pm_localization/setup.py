from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'pm_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mujin',
    maintainer_email='kazuho.kobayashi.ynu@gmail.com',
    description='localization for patasmonkey UGV',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wheel_odometry_node = pm_localization.wheel_odometry_node:main',
            'odom_to_path_node = pm_localization.odom_to_path_node:main',
            'vio_odom_adapter_node = pm_localization.vio_odom_adapter_node:main',
            'vio_vertical_gate_node = pm_localization.vio_vertical_gate_node:main',
            'vio_twist_gate_node = pm_localization.vio_twist_gate_node:main',
            'local_odometry_composer_node = pm_localization.local_odometry_composer_node:main',
            'attitude_height_observer_node = pm_localization.attitude_height_observer_node:main',
            'heading_initializer_node = pm_localization.heading_initializer_node:main',
            'gnss_fix_gate_node = pm_localization.gnss_fix_gate_node:main',
        ],
    },
)
