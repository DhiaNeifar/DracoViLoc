from glob import glob

from setuptools import find_packages, setup


package_name = 'isaac_ros_yolo_direction'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dhianeifar',
    maintainer_email='dhianeifar@users.noreply.github.com',
    description='Project YOLO bounding-box centers into camera-frame direction rays.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'direction_publisher = isaac_ros_yolo_direction.direction_publisher:main',
        ],
    },
)
