from setuptools import setup
import os
from glob import glob
p = 'throwbot_sensors'
setup(
    name=p, version='0.1.0', packages=[p],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + p]),
        ('share/' + p, ['package.xml']),
        (os.path.join('share', p, 'launch'), glob('launch/*.py')),
        (os.path.join('share', p, 'urdf'), glob('urdf/*')),
        (os.path.join('share', p, 'worlds'), glob('worlds/*')),
        (os.path.join('share', p, 'config'), glob('config/*')),
        (os.path.join('share', p, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    entry_points={'console_scripts': [
        'obstacle_mover = throwbot_sensors.obstacle_mover:main',
        'range_from_scan = throwbot_sensors.range_from_scan:main',
        'study_logger = throwbot_sensors.study_logger:main',
    ]},
)
