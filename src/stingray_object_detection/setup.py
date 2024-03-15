from setuptools import setup

package_name = 'stingray_object_detection'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name, "yolov5"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'yolo_detector = stingray_object_detection.yolo_detector:main',
        ],
    },
)
