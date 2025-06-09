# Copyright (c) 2025 Boston Dynamics AI Institute LLC.  All rights reserved.

from setuptools import find_packages, setup

package_name = "spot_common"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["py.typed"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    maintainer="The AI Institute",
    maintainer_email="opensource@theaiinstitute.com",
    description="Common utilities in the Spot ROS 2 stack",
    tests_require=["pytest"],
    zip_safe=True,
    license="MIT",
)
