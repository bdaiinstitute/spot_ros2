from setuptools import setup

package_name = "simple_arm_motion"

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
    description="Direct conversion of BD simple arm command example",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_simple = simple_arm_motion.arm_simple:main",
        ],
    },
)
