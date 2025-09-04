from setuptools import setup
package_name = 'blue_segmentation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/blue_seg.launch.py']),
        ('share/' + package_name + '/launch', ['launch/follow_blue_go2.launch.py']),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Realtime blue segmentation node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'blue_filter_node = blue_segmentation.blue_filter_node:main',
            'blue_follower_node = blue_segmentation.blue_follower_node:main',
            'robot_driver_node = blue_segmentation.robot_driver_node:main',
        ],
    },
)
