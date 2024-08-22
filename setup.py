from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'force_torque_sensor'


from generate_parameter_library_py.setup_helper import generate_parameter_module

generate_parameter_module(
  "calibration_node_parameters", # python module name for parameter library
  "force_torque_sensor/calibration_node_parameters.yaml", # path to input yaml file
)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.xml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools',
                      'generate_parameter_library'],
    zip_safe=True,
    maintainer='Nikola Banovic',
    maintainer_email='nibanovic@gmail.com',
    description='calibration for robot tool',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_node = force_torque_sensor.calibrate_tool:main'
        ],
    },
)
