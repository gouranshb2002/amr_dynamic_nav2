import os
from glob import glob
from setuptools import find_packages, setup

setup(
    name='amr_benchmark',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/amr_benchmark']),
        ('share/amr_benchmark', ['package.xml']),
        (os.path.join('share', 'amr_benchmark', 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': ['run_static_pilot = amr_benchmark.run_static_pilot:main', 'run_dynamic_benchmark = amr_benchmark.run_dynamic_benchmark:main']},
)
