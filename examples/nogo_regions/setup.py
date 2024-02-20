from setuptools import setup

package_name = "nogo_regions"

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
    maintainer="The AI Institute",
    maintainer_email="engineering@theaiinstitute.com",
    description="Example of using no-go regions with Spot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "nogo_example = nogo_regions.nogo_example:main",
        ],
    },
)
