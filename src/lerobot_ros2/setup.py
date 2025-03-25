import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lerobot_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'lerobot_ros2/launch'), glob('launch/*')),
        (os.path.join('share', 'lerobot_ros2/config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lio',
    maintainer_email='lioSgr1205@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lerobot_ros2 = lerobot_ros2.lerobot_ros2:main',
            # 'lerobot_ros2 = lerobot_ros2.lerobot_shell:main',
        ],
    },
)
