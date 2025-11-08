import glob

from setuptools import find_packages, setup

package_name = "tello_competition"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xtal",
    maintainer_email="manuel_lego1234@hotmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            'camera_shot = tello_competition.camera_shot:main',
            'camera_calibration = tello_competition.camera_calibration:main',
            'save_slam_map = tello_competition.save_slam_map:main',
            'visualize_slam_map = tello_competition.visualize_slam_map:main '
        ],
    },
)
