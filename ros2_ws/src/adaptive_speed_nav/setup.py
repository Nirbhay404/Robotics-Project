import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'adaptive_speed_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Nirbhay Kumar (24BCS089), Nikhil Kumar Sinha (24BCS086), Naman Kumar (24BCS080)',
    maintainer_email='194455286+Nirbhay404@users.noreply.github.com',
    description='Adaptive speed control in cluttered environments',
    license='MIT',
    entry_points={
        'console_scripts': [
            'navigator = adaptive_speed_nav.navigator_node:main',
        ],
    },
)
