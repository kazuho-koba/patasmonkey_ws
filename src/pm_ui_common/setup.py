from glob import glob
from setuptools import find_packages, setup

setup(
    name='pm_ui_common', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/pm_ui_common']),
                ('share/pm_ui_common', ['package.xml', 'README.md']),
                ('share/pm_ui_common/launch', glob('launch/*.launch.py')),
                ('share/pm_ui_common/config', glob('config/*.yaml'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='kazuho', maintainer_email='kazuho.kobayashi.ynu@gmail.com',
    description='Patasmonkey Qt地図表示の共通部品', license='MIT',
    entry_points={'console_scripts': []},
)
