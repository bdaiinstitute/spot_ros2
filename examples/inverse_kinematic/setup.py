from setuptools import setup

package_name = "inverse_kinematic_requests"

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
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "send_inverse_kinematic_requests = inverse_kinematic_requests.send_inverse_kinematic_requests:main",
        ],
    },
)
