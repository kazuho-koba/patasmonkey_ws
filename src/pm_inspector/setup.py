from glob import glob
from setuptools import find_packages, setup

setup(name='pm_inspector', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/pm_inspector']),
                  ('share/pm_inspector', ['package.xml', 'README.md']),
                  ('share/pm_inspector/launch', glob('launch/*.py')),
                  ('share/pm_inspector/config', glob('config/*')),
                  ('share/pm_inspector/systemd', glob('systemd/*'))],
      install_requires=['setuptools'], zip_safe=True, maintainer='kazuho',
      maintainer_email='kazuho.kobayashi.ynu@gmail.com', license='MIT',
      description='停止済みbagのローカル・遠隔検査アプリ',
      entry_points={'console_scripts': ['bag_inspector = pm_inspector.app:main',
                                        'inspection_agent = pm_inspector.agent:main']})
