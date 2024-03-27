from setuptools import find_packages, setup
from glob import glob

package_name = 'stingray_object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']) + find_packages(where='ultralytics', include=["ultralytics", "ultralytics.*"]),
    package_dir={'ultralytics': 'ultralytics/ultralytics'},
    package_data={ "ultralytics": ["**/*.yaml"], "ultralytics.assets" : ["*.jpg"] },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/weights', glob('weights/*.pt')),
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
            'yolov5_detector = stingray_object_detection.yolov5_detector:main',
            'yolov8_detector = stingray_object_detection.yolov8_detector:main',
        ],
    },
)
