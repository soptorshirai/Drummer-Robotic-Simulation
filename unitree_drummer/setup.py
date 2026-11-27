from setuptools import setup

PACKAGE = 'unitree_drummer'

setup(
    name=PACKAGE,
    version='0.1.0',
    packages=[PACKAGE],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{PACKAGE}']),
        (f'share/{PACKAGE}', ['package.xml']),
        (f'share/{PACKAGE}/launch', ['launch/drummer.launch.py']),
        (f'share/{PACKAGE}/config', ['config/test_groove.json']),
    ],
    entry_points={
        'console_scripts': ['drummer = unitree_drummer.trajectory_node:main'],
    },
)
