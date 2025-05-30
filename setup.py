import os
from glob import glob

from setuptools import find_packages, setup

package_name = "turtlesim_agent"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.xml")),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="yutarop.storm.7@gmail.com",
    description="TurtleSim Agent is an AI agent for the turtlesim simulator that interprets text instructions and transforms them into visual outputs using LangChain-turning the simulated turtle into a creative artist",
    license="MIT License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["turtlesim_agent_node = turtlesim_agent.main:main"],
    },
)
