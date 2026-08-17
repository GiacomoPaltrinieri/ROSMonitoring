from glob import glob
from setuptools import find_packages, setup


package_name = "nav_arm_case_study"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Giacomo",
    maintainer_email="giaco.paltri@example.com",
    description="Nav2/MoveIt interlock case study for ROSMonitoring",
    license="MIT",
    entry_points={
        "console_scripts": [
            "scenario_runner = nav_arm_case_study.scenario_runner:main",
            "box_scenario = nav_arm_case_study.box_scenario:main",
            "joint_state_sampler = nav_arm_case_study.joint_state_sampler:main",
        ],
    },
)
