from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hopejr_real_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jisu',
    maintainer_email='sjs15290@knu.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = hopejr_real_bridge.real_bridge_node:main',
            'motor_state_pub = hopejr_real_bridge.motor_state_publisher:main',
            'joint_state_estimator = hopejr_real_bridge.joint_state_estimator:main',
            'calibration = hopejr_real_bridge.calibration:main',
        ],
    },
)
