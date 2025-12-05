import os
from glob import glob
from setuptools import find_packages, setup

package_name = "state_machine"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (os.path.join("share", package_name, "maps"), glob(os.path.join("maps", "*"))),
        (os.path.join("share", package_name, "rviz"), glob(os.path.join("rviz", "*"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="juan",
    maintainer_email="jkaplan@udesa.edu.ar",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "map_publisher = state_machine.map_publisher:main",
            "likelihood_map_publisher = state_machine.likelihood:main",
            "localization = state_machine.localization:main",
        ],
    },
)
