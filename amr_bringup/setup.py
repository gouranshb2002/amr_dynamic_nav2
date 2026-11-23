import os
from glob import glob
from setuptools import find_packages, setup

setup(
    name='amr_bringup',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/amr_bringup']),
        ('share/amr_bringup', ['package.xml']),
        (os.path.join('share', 'amr_bringup', 'launch'), glob('launch/*.py')),
        (os.path.join('share', 'amr_bringup', 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gouransh Bhatnagar',
    maintainer_email='gouranshb2002@users.noreply.github.com',
    description='Simulation bringup and safety watchdog for amr_dynamic_nav2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cmd_vel_watchdog = amr_bringup.cmd_vel_watchdog:main',
        ],
    },
)
