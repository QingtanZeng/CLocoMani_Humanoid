import os
from glob import glob
from setuptools import setup

package_name = "wbDynPlan_pyutils"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Qingtan Zeng",
    maintainer_email="zengqt.e@gmail.com",
    description="Ros2 MPC python utils",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mpc_observation_logger = wbDynPlan_pyutils.mpc_observation_logger:main",
        ],
    },
)
