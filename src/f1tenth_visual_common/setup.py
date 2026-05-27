from setuptools import find_packages, setup


package_name = "f1tenth_visual_common"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/dashboard.launch.py",
            ],
        ),
        (
            f"share/{package_name}/config",
            ["config/topics.yaml", "config/track.yaml", "config/waypoints_map.csv"],
        ),
        (f"share/{package_name}/foxglove", ["foxglove/layout_f1tenth_gym.json"]),
        (f"share/{package_name}", ["README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="F1TENTH Team",
    maintainer_email="student@example.com",
    description="Foxglove template and live driving statistics nodes for F1TENTH Gym.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "trajectory_node = f1tenth_visual_common.trajectory_node:main",
            "stats_node = f1tenth_visual_common.stats_node:main",
        ],
    },
)
