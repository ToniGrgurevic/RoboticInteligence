from setuptools import setup
from glob import glob

package_name = 'tilde'

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch/", glob("launch/*launch*")),
        ('share/' + package_name + "/rviz/", glob("rviz/*")),
        ('share/' + package_name + "/world/" , glob('world/*')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chris',
    maintainer_email='christopher.schneider@gmx.net',
    description='Ros2 simulation of 2 robots following a wall and robot in front',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # Declare the enry point for your python scripts
    entry_points={
        'console_scripts': [
            'tilde = tilde.__init__:main'
        ],
    },
)
