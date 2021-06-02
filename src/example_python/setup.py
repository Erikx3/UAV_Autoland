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
            'helloworld_node = example_python.helloworld_node:main'
        ],
    },
)
