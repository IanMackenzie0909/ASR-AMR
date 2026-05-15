"""Packaging configuration for the object tracking ROS 2 Python nodes."""

from glob import glob

from setuptools import find_packages, setup

package_name = 'object_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            glob('config/*.json')),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
        ('share/' + package_name + '/' + package_name,
            glob(package_name + '/*.pt') + glob(package_name + '/*.json')),
        ('share/' + package_name + '/' + package_name + '/pbl/docs',
            glob(package_name + '/pbl/docs/*.md')),
    ],
    install_requires=['setuptools'],
    package_data={
        package_name: ['best.pt', 'hsv_values.json'],
    },
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'object_tracking_img = object_tracking.object_tracking_img:main',
            'hsv_adjust = object_tracking.hsv_adjust:main',
            'robot_task = object_tracking.robot_task:main', # Removed the space here
            'object_tracking_YOLO = object_tracking.object_tracking_YOLO:main',
            'pbl_inspection = object_tracking.pbl.inspection_node:main',
            'pbl_vision_api = object_tracking.pbl.vision_api_node:main'
        ],
    },
    
 )
