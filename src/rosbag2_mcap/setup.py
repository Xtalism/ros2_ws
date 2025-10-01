from setuptools import find_packages, setup

package_name = 'rosbag2_mcap'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xtal',
    maintainer_email='manuel_lego1234@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'read_rosbag2 = rosbag2_mcap.read_rosbag2:main',
            'csv_rosbag2 = rosbag2_mcap.csv_rosbag2:main',
            'graph_rosbag2 = rosbag2_mcap.graph_rosbag2:main',
        ],
    },
)
