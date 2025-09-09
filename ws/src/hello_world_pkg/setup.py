from setuptools import find_packages, setup

package_name = 'hello_world_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jgiraldo29',
    maintainer_email='jmg765@miami.edu',
    description='A simple talker test package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = hello_world_pkg.talker:main',
        ],
    },
)