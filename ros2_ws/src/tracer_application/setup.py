from setuptools import find_packages, setup

package_name = 'tracer_application'

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
    maintainer='bing6866',
    maintainer_email='b1417853077@outlook.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'init_robot_pose=tracer_application.init_robot_pose:main',
            'get_robot_pose=tracer_application.get_robot_pose:main',
            'nav_to_pose=tracer_application.nav_to_pose:main',
            'waypoint_follower=tracer_application.waypoint_follower:main',

            
        ],
    },
)
