"""Python module・設定・説明用データをFoxy overlayへ配置する。"""
from glob import glob
from setuptools import find_packages, setup

setup(name='pm_planner', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/pm_planner']),
                  ('share/pm_planner', ['package.xml', 'README.md']),
                  ('share/pm_planner/launch', glob('launch/*.launch.py')),
                  ('share/pm_planner/config', glob('config/*.yaml')),
                  ('share/pm_planner/examples', glob('examples/*.yaml')),
                  ('share/pm_planner/rviz', glob('rviz/*.rviz'))],
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
      maintainer='kazuho', maintainer_email='kazuho.kobayashi.ynu@gmail.com', license='MIT',
      description='Patasmonkeyの非駆動参照経路とミッション進捗判定',
      entry_points={'console_scripts': ['planner_preview = pm_planner.preview_node:main',
                                        'preview_mission = pm_planner.cli:main']})
