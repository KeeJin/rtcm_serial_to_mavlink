from setuptools import setup
from glob import glob


package_name = "rtcm_serial_to_mavlink"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial>=3.5,<4.0", "pyrtcm>=1.1.0,<2.0"],
    zip_safe=True,
    maintainer="maintainer",
    maintainer_email="maintainer@example.com",
    description="RTCM tools including a ROS 2 node publishing mavros_msgs/RTCM.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rtcm_to_mavros_node = rtcm_serial_to_mavlink.rtcm_to_mavros_node:main",
        ],
    },
)
