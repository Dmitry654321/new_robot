from setuptools import setup

package_name = "tof_stop_on_obstacle"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Stops Duckiebot when ToF detects obstacle; LEDs show state.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "stop_on_obstacle = tof_stop_on_obstacle.stop_on_obstacle:main",
        ],
    },
)
