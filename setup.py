from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # ✅ launch file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Pure pursuit path tracking',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = pure_pursuit.scripts.pure_pursuit_node:main',
            'interpolator = pure_pursuit.scripts.path_interpolator_node:main',
            'race_line = pure_pursuit.scripts.race_line:main',
        ],
    },
)
