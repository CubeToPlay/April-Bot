from setuptools import find_packages, setup

package_name = 'goal'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/goal_launch.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='awan0888',
    maintainer_email='awan.1389a@gmail.com',
    description='Takes gesture information, and goal information and pubishes a goal_id',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'goal = goal.goal:main'
        ],
    },
)
