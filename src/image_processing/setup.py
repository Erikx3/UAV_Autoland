import os
from glob import glob
from setuptools import setup

package_name = 'image_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Erik Suer',
    maintainer_email='e.suer@tu-berlin.de',
    description='Image Processing Package Python',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_vector_node = image_processing.image_vector_node:main',
        ],
    },
)
