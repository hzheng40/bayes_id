from setuptools import setup
import os
from glob import glob

package_name = 'traj_collector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hongrui Zheng',
    maintainer_email='billyzheng.bz@gmail.com',
    description='Trajectory collector for F1TENTH vehicles.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collector = traj_collector.collector:main',
        ],
    },
)
