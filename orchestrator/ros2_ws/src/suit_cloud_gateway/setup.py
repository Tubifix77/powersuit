from setuptools import find_packages, setup

package_name = "suit_cloud_gateway"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/links.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Powersuit Team",
    maintainer_email="twb@systematic.com",
    description="Node 8 to Node 9 TLS WebSocket link client",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gateway_node = suit_cloud_gateway.gateway_node:main",
        ],
    },
)
