from setuptools import find_packages, setup

package_name = 'track_parrot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/get_keypoint_launch.py',
                                               'launch/pursuer_launch_yolo.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yousa',
    maintainer_email='yousa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collect_parrot_fig = track_parrot.collect_parrot_fig:main',
            'get_3d_bbox_yolo = track_parrot.get_3d_bbox_yolo:main',
            'get_3d_pos_yolo = track_parrot.get_3d_pos_yolo:main',
            'get_anafi_vedio = track_parrot.get_anafi_vedio:main',
            'get_drones_state = track_parrot.get_drones_state:main',
            'get_keypoints = track_parrot.get_keypoints:main',
            'get_vicon_data = track_parrot.get_vicon_data:main',
            'plot_data_yolo = track_parrot.plot_data_yolo:main',
            'tracking_drone_yolo = track_parrot.tracking_drone_yolo:main',
        ],
    },
)
