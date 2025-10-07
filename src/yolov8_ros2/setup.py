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
    maintainer_email='ldl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #!/home/ldl/anaconda3/envs/yolov8/bin/python
            'yolov8_node = yolov8_ros2.yolov8_node:main'
        ],
    },
)
