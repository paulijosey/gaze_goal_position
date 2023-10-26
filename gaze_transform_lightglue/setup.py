from setuptools import find_packages, setup
import os  
from glob import glob

package_name = 'gaze_transform_lightglue'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paul Joseph',
    maintainer_email='josephp@ethz.ch',
    description='TODO: Package description',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gaze_transform_node = gaze_transform_lightglue.gaze_transform_node:main'
        ],
    },
)
