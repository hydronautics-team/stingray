import glob
import os

from setuptools import find_packages, setup

package_name = "stingray_localization"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob.glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="waspishraccoon",
    maintainer_email="p.v.elesin@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "auv_odometry_tf2_broadcaster = stingray_localization.auv_odometry_tf2_broadcaster:main",
            "test = stingray_localization.test:main",
        ],
    },
)
