import os
from glob import glob
from setuptools import setup

package_name = 'ciic_ros_tasks'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bruno Rocha',
    description='Intelligent task distribution system',
    maintainer_email='bruno.c.rocha@ipleiria.pt',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_car_teleop = ciic_ros_tasks.task_car_teleop:main',
            'tasks_booker = ciic_ros_tasks.tasks_booker:main',
            'task_car_performer = ciic_ros_tasks.task_car_performer:main'
        ],
    },
)
