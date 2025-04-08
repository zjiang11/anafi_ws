from setuptools import find_packages, setup

package_name = 'anafi_test'

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
    maintainer='yousa',
    maintainer_email='zjiang11@ualberta.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collect_anafi_data_linear_mpc = anafi_test.collect_anafi_data_linear_mpc:main',
            'collect_anafi_data_newton_euler_mpc = anafi_test.collect_anafi_data_newton_euler_mpc:main',
            'manual_control = anafi_test.manual_control:main', 
            'move2point_linear_mpc = anafi_test.move2point_linear_mpc:main',
            'move2point_newton_euler_mpc = anafi_test.move2point_newton_euler_mpc:main'
        ],
    },
)
