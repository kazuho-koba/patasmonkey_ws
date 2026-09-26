from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'pm_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'desktop'), glob('desktop/*.desktop.in')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kazuho',
    maintainer_email='kazuho.kobayashi.ynu@gmail.com',
    description='Patasmonkey UGV operator console',
    license='MIT',
    entry_points={
        'console_scripts': [
            'operator_console = pm_gui.operator_console:main',
        ],
    },
)
