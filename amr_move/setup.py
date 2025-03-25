from setuptools import find_packages, setup

package_name = 'amr_move'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/amr_move/launch', ['launch/amr_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey2',
    maintainer_email='rokey2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'init_pose = amr_move.init_pose:main',
            'send_init = amr_move.send_init:main',
            'third = amr_move.send_waypoint:main',
            'last_send = amr_move.send_inway_1:main',
            'last_activate = amr_move.init_waypoint:main',
            'please = amr_move.why_not_4:main',
        ],
    },
)
