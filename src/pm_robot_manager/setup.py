from setuptools import find_packages, setup

package_name = 'pm_robot_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', ['config/robot_manager.yaml']),
        ('share/' + package_name + '/systemd',
         ['systemd/pm-robot-manager.service',
          'systemd/pm-robot-manager.sudoers',
          'systemd/start-pm.service',
          'systemd/pm-mission-bag.service',
          'systemd/pm-debug-bag.service']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kazuho',
    maintainer_email='kazuho.kobayashi.ynu@gmail.com',
    description='Robot Core systemd manager',
    license='MIT',
    entry_points={
        'console_scripts': [
            'robot_manager = pm_robot_manager.robot_manager:main',
            'wait_bag_clock = pm_robot_manager.wait_bag_clock:main',
            'stop_bag_wait = pm_robot_manager.stop_bag_wait:main',
        ],
    },
)
