from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pupil_labs_glasses'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paul Joseph',
    maintainer_email='josephp@ethz.ch',
    description='This pkg lets you connect to a PupilLabs' + 
                'Glasses device and publishes its data to ROS2 topics',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pupil_labs_stream_node = pupil_labs_glasses.pupil_labs_stream_node:main'
        ],
    },
)
