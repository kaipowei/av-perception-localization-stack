from setuptools import find_packages, setup

package_name = "fa_perception_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kai-Po Wei",
    maintainer_email="kaipowei@umich.edu",
    description="Camera-based 2D object detection (YOLO)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "yolo_detector_node = fa_perception_py.yolo_detector_node:main",
        ],
    },
)
