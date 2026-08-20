import os
from glob import glob

from setuptools import find_packages, setup

package_name = "geo_tuner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mohamed Abdelkader",
    maintainer_email="mohamedashraf123@gmail.com",
    description="Gain design and safe in-flight auto-tuning for the "
                "mav_controllers_ros geometric controller",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tuning_conductor = geo_tuner.tuning_conductor:main",
            "quad_sim = geo_tuner.quad_sim:main",
            "geo-tuner-design = geo_tuner.cli.design_gains:main",
            "geo-tuner-hover = geo_tuner.cli.analyze_hover:main",
        ],
    },
)
