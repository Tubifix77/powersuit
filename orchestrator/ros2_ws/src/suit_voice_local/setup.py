from setuptools import find_packages, setup

package_name = "suit_voice_local"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/grammar.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Powersuit Team",
    maintainer_email="twb@systematic.com",
    description="Offline voice intent matching and acknowledgment tones",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "voice_node = suit_voice_local.voice_node:main",
        ],
    },
)
