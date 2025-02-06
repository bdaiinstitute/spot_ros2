from setuptools import setup

package_name = "spot_examples"

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
    description="Examples of using ROS 2 to control Spot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "walk_forward = spot_examples.walk_forward:main",
            "arm_simple = spot_examples.arm_simple:main",
            "send_inverse_kinematics_requests = spot_examples.send_inverse_kinematics_requests:main",
            "batch_trajectory = spot_examples.batch_trajectory:main",
            "hello_spot = spot_examples.hello_spot:main",
            "arm_with_body_follow = spot_examples.arm_with_body_follow:main",
            "wasd = spot_examples.wasd:main",
        ],
    },
)
