from setuptools import setup

package_name = 'pegasus_api'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marcelojacinto',
    maintainer_email='marcelo.jacinto@tecnico.ulisboa.pt',
    description='TODO: Package description',
    license='BSD 3-Clause License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
