from setuptools import setup

package_name = "lathe_arm_teach"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_data={package_name: ["hmi.html"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/waypoints", ["waypoints/example_load_cycle.yaml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=False,
    maintainer="Maharaj L M R",
    maintainer_email="lmrrajan2005@gmail.com",
    description="Potentiometer teleop, web HMI and point replay for the jib arm.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "replay_node = lathe_arm_teach.replay_node:main",
            "pot_teleop = lathe_arm_teach.pot_teleop:main",
            "hmi_server = lathe_arm_teach.hmi_server:main",
            "linkage_node = lathe_arm_teach.linkage_node:main",
            "part_grasp = lathe_arm_teach.part_grasp:main",
        ],
    },
)
