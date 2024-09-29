from setuptools import setup

package_name = 'guided_landing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'pupil-apriltags'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='konudroid@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'track_and_land = guided_landing.track_and_land:main',
            'tag_detect = guided_landing.apriltag_detector_node:main',
            'pid_control = guided_landing.pid_controller_node:main',
            'sim_tag_detect = guided_landing.apriltag_detector_sim_node:main',
            'sim_pid_control = guided_landing.pid_controller_sim_node:main',
            'marker_control = guided_landing.marker_control_node:main',
        ],
    },
)
