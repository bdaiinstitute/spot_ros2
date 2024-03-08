from setuptools import setup

package_name = "batch_trajectory"

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
    description="Example of long trajectory batching",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "batch_trajectory_example = batch_trajectory_example.batch_trajectory:main",
        ],
    },
)
