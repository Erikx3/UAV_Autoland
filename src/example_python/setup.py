import os
from glob import glob
from setuptools import setup

package_name = 'example_python'

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
    description='Example Package Python',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_helloworld_node = example_python.helloworld_node:main',
            'example_talker = example_python.publisher_member_function:main',
            'example_listener = example_python.subscriber_member_function:main',
            'example_talk_lis = example_python.subpub_member_function:main',
        ],
    },
)
