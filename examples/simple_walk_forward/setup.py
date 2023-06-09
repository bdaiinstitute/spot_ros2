from setuptools import setup

package_name = "simple_walk_forward"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BD AI Institute",
    maintainer_email="engineering@theaiinstitute.com",
    description="Simple walk forward example using ROS2",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "walk_forward = simple_walk_forward.walk_forward:main",
        ],
    },
)
