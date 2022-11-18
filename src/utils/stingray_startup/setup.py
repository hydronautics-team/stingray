from setuptools import setup

package_name = 'stingray_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vladushked',
    maintainer_email='vladushked@yandex.ru',
    description='The stingray_vision package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'opencv_object_detector = stingray_vision.opencv_object_detector:main',
        ],
    },
)