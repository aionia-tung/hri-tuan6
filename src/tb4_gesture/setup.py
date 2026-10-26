from glob import glob

from setuptools import setup

package_name = 'tb4_gesture'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/worlds', glob('worlds/*.sdf')),
        (f'share/{package_name}/models', glob('models/*.task')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aionia-tung',
    maintainer_email='tst159205@gmail.com',
    description='Hand gesture recognition (MediaPipe) controlling TurtleBot 4 via ROS 2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_recognition_node = tb4_gesture.gesture_recognition_node:main',
            'robot_behavior_node = tb4_gesture.robot_behavior_node:main',
            'webcam_test = tb4_gesture.webcam_test:main',
            'gesture_replay = tb4_gesture.gesture_replay:main',
        ],
    },
)
