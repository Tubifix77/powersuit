import os
from glob import glob

from setuptools import find_packages, setup

package_name = "suit_state_estimation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Powersuit Team",
    maintainer_email="twb@systematic.com",
    description="Joint-state aggregation, torso IMU relay and EKF configuration",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "joint_state_aggregator = suit_state_estimation.joint_state_aggregator:main",
            "imu_selector = suit_state_estimation.imu_selector:main",
        ],
    },
)
