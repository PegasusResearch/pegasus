import os
from glob import glob
from setuptools import setup

package_name = 'mocap_simulator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
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
    description='A simple simulator for a mocap pose',
    license='BSD 3-Clause License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mocap_simulator = mocap_simulator.mocap_simulator_node:main'
        ],
    },
)
