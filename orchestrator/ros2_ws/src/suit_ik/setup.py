from setuptools import find_packages, setup

package_name = "suit_ik"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Powersuit Team",
    maintainer_email="twb@systematic.com",
    description="Planar 2-link limb IK and flight-surface mapper for the powersuit",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ik_node = suit_ik.ik_node:main",
            "flight_mapper = suit_ik.flight_mapper:main",
        ],
    },
)
