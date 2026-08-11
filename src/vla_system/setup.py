from glob import glob
from setuptools import find_packages, setup

package_name = "vla_system"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/models", glob("models/*.pt")),
        ("share/" + package_name + "/models/yolo26s-seg_openvino_model", glob("models/yolo26s-seg_openvino_model/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="VLA Project",
    maintainer_email="maintainer@example.com",
    description="Conversational LLM-driven picking system for M0609 + RG2",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "perception_node = vla_system.nodes.perception_node:main",
            "agent_node = vla_system.nodes.agent_node:main",
            "vla_pick_bridge_node = vla_system.nodes.vla_pick_bridge_node:main",
            "table_homography_test = vla_system.nodes.table_homography_test_node:main",
            "vla_gui = vla_system.vla_gui:main",
        ]
    },
)
