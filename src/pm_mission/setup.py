from glob import glob
from setuptools import find_packages, setup

setup(
    name='pm_mission', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/pm_mission']),
                ('share/pm_mission', ['package.xml', 'README.md']),
                ('share/pm_mission/launch', glob('launch/*.launch.py')),
                ('share/pm_mission/config', glob('config/*.yaml'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='kazuho', maintainer_email='kazuho.kobayashi.ynu@gmail.com',
    description='Patasmonkeyミッション編集・受領アプリ', license='MIT',
    entry_points={'console_scripts': ['mission_planner = pm_mission.app:main', 'mission_receiver = pm_mission.receiver:main']},
)
