from setuptools import find_packages, setup

package_name = "twinguard_dataset_replay"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nitin",
    maintainer_email="nitin@example.com",
    description="ROS 2 dataset replay bridge for TwinGuard PX4/Gazebo validation.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "dataset_replay_node = twinguard_dataset_replay.dataset_replay_node:main",
        ],
    },
)
