from setuptools import setup

package_name = "set_nogo_region"

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
    description="Setting a no-go region for Spot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "set_nogo = set_nogo_region.nogo_region:main",
        ],
    },
)
