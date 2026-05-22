import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tb3_bringup'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.srdf')),
        (os.path.join('share', package_name, 'config', 'rviz'),
            glob('config/rviz/*.rviz')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juanks',
    maintainer_email='juankibenitez57@gmail.com',
    description='Stack completo de navegación autónoma TurtleBot3.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': []},
)
