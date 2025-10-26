from setuptools import setup

package_name = 'yolov8_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ldl',
    maintainer_email='3082128164@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov8_node = yolov8_ros2.yolov8_node:main',
            'yolov8_depth_node = yolov8_ros2.yolov8_depth_node:main',
            'yolov8_distance_node = yolov8_ros2.yolov8_distance_node:main',
        ],
    },
)
