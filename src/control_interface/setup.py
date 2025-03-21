import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'control_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'control_interface/launch'), glob('launch/*')),
        (os.path.join('share', 'control_interface/config'), glob('config/*')),
        (os.path.join('share', 'control_interface/doc'), glob('doc/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lio',
    maintainer_email='liuweinan@fcbox.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_interface = control_interface.control_interface_shell:main'
        ],
    },
)
