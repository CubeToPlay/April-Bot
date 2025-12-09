from setuptools import find_packages, setup
import os
import glob

package_name = 'april_bot_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/worlds', 
            ['worlds/apriltag_world.sdf']),
        ('share/' + package_name + '/launch', 
            ['launch/april_world_turtlebot3.launch.py']),
        ('share/' + package_name, ['package.xml']),
        # https://stackoverflow.com/questions/27829754/include-entire-directory-in-python-setup-py-data-files
        ('share/' + package_name + '/models', 
            glob.glob('models/april_tag_models/*')),
        ('share/' + package_name + '/models', 
            glob.glob('models/rock_test/*')),
        ('share/' + package_name + '/models', 
            ['models/worldwalls.dae']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='awan0888',
    maintainer_email='awan0888@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
