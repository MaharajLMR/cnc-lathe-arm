from setuptools import setup

package_name = "lathe_arm_teach"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/waypoints", ["waypoints/example_load_cycle.yaml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="Maharaj L M R",
    maintainer_email="lmrrajan2005@gmail.com",
    description="Teach mode: jog arm and save waypoints to YAML. Replay via MoveIt 2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "teach_node  = lathe_arm_teach.teach_node:main",
            "replay_node = lathe_arm_teach.replay_node:main",
        ],
    },
)
