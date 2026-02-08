from setuptools import setup
import os
from glob import glob

package_name = 'tb_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'robots'), glob('config/robots/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shaon',
    maintainer_email='shaon@gmail.com',
    description='Bringup launch files for Mini TurtleBot swarm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
