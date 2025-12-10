from setuptools import setup

package_name = "humanoid_common_mpc_ros2"

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
    maintainer="Qingtan Zeng",
    maintainer_email="zengqt.e@gmail.com",
    description="Ros2 interface for the humanoid common mpc",
    license="TODO: License declaration",
    tests_require=["pytest"],
)
