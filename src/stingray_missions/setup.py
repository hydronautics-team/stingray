from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'stingray_missions'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/configs/missions', glob('configs/missions/*.yaml')),
        (f'share/{package_name}/configs/scenarios', glob('configs/scenarios/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vladushked',
    maintainer_email='vladik1209@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm_node = stingray_missions.fsm_node:main',
        ],
    },
)
