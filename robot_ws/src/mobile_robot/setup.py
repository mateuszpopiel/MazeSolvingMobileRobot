import os
from glob import glob
from setuptools import setup

package_name = 'mobile_robot'

setup(
  name=package_name,
  version='0.0.0',
  packages=[package_name],
  py_modules=[
    'mobile_robot.lib.lights_manager',
    'mobile_robot.lib.line_detector',
  ],
  data_files=[
      ('share/ament_index/resource_index/packages',
          ['resource/' + package_name]),
      ('share/' + package_name, ['package.xml']),
      (os.path.join('share', package_name), glob('launch/*.launch.py')),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='Mateusz Popiel',
  maintainer_email='popielmateusz12@gmail.com',
  description='TODO: Package description',
  license='MIT',
  tests_require=['pytest'],
  entry_points={
    'console_scripts': [
      'maze_solver = mobile_robot.maze_solver:main',
      'line_detector = mobile_robot.line_detector_node:main',
    ],
  },
)
