from setuptools import find_packages, setup

package_name = "suit_blackbox"

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
    description="mmap ring flight recorder with snapshot-on-estop",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "blackbox_node = suit_blackbox.blackbox_node:main",
        ],
    },
)
