import os
from glob import glob
from setuptools import setup

package_name = 'ueye_driver'
submodules = package_name + "/ueye"

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marcelo Jacinto',
    maintainer_email='marcelo.jacinto@tecnico.ulisboa.pt',
    description='A driver for publishing video feed from ueye cameras to ROS 2 images',
    license='BSD 3-Clause License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ueye_driver = ueye_driver.ueye_driver_node:main'
        ],
    },
)
