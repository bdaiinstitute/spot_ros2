from setuptools import setup

package_name = "inverse_kinematics_example"

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
    maintainer_email="gremigi@theaiinstitute.com",
    description="Example of inverse kinematic requests.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "send_inverse_kinematics_requests = inverse_kinematics_example.send_inverse_kinematics_requests:main",
        ],
    },
)
