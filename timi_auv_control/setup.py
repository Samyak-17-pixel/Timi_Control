from setuptools import find_packages, setup

package_name = 'timi_auv_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/auv_control.launch.py']),
        ('share/' + package_name + '/config', [
            'config/control_params.yaml',
            'config/geometry.yaml',
            'config/missions/README.md',
        ]),
        ('share/' + package_name + '/config/missions/station_keeping', [
            'config/missions/station_keeping/mission.yaml',
            'config/missions/station_keeping/README.md',
        ]),
        ('share/' + package_name + '/config/missions/waypoint', [
            'config/missions/waypoint/mission.yaml',
            'config/missions/waypoint/README.md',
        ]),
        ('share/' + package_name + '/config/missions/path_following', [
            'config/missions/path_following/mission.yaml',
            'config/missions/path_following/README.md',
        ]),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML'],
    zip_safe=True,
    maintainer='Timi AUV',
    maintainer_email='timi_auv@example.com',
    description='6-DOF AUV ROS 2 control with missions',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mother_node = timi_auv_control.mother_node:main',
        ],
    },
)
