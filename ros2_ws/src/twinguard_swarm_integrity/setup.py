from setuptools import find_packages, setup

package_name = "twinguard_swarm_integrity"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Nitin",
    maintainer_email="nitin@example.com",
    description="Digital twin integrity, trust, and resilient formation logic for UAV swarms.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "integrity_node = twinguard_swarm_integrity.integrity_node:main",
            "offboard_mission_controller = twinguard_swarm_integrity.offboard_mission_controller:main",
        ],
    },
)
