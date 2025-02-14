import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pegasus_demos'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marcelo Jacinto',
    maintainer_email='mjacinto@isr.tecnico.ulisboa.pt',
    description='Demos for the Pegasus project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pegasus_demos = pegasus_demos.demo:main',
        ],
    },
)
