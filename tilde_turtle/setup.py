from setuptools import setup
from glob import glob

package_name = 'tilde_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch/", glob("launch/*launch*")),
        ('share/' + package_name + "/rviz/", glob("rviz/*")),
        ('share/' + package_name + "/tilde_world/" , glob('tilde_world/*')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christopher Schneider, Toni Grgurevic, Juan Camilo Pacheco',
    maintainer_email='up202401397@edu.fe.up.pt',
    description='Package description',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follower_slow = tilde_turtle.wall_follower_slow:main',
            'wall_follower_fast = tilde_turtle.wall_follower_fast:main'
        ],
    },
)