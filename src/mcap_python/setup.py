from setuptools import find_packages, setup

package_name = 'mcap_python'

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
            'read_rosbag2 = mcap_python.read_rosbag2:main',
            'csv_rosbag2 = mcap_python.csv_rosbag2:main',
            'graph_rosbag2 = mcap_python.graph_rosbag2:main',
        ],
    },
)
