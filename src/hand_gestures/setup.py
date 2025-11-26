from setuptools import find_packages, setup

package_name = 'hand_gestures'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource',
            ['resource/hand_landmarker.task']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/hand_gestures_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adam',
    maintainer_email='hwvxyeej@gmail.com',
    description='Peforms gesture detection with the webcam',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'webcam = hand_gestures.webcam:main',
            'gestures = hand_gestures.gestures:main'
        ],
    },
)
