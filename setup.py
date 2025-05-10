from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'webots_ros2_omni'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*_launch.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.wbt")),
        (os.path.join("share", package_name, "protos", "meshes","omni"), glob("protos/meshes/omni/*")),

        (os.path.join("share", package_name, "resource"), glob("resource/*")), 
        (os.path.join("share", package_name, "protos"), glob("protos/*.proto")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guts',
    maintainer_email='ninad.kulkarni@alumni.fh-aachen.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_sync = webots_ros2_omni.robot_sync:main',
        ],
    },
)
