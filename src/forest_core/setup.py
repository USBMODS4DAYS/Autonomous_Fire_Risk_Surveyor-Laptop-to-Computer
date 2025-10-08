from setuptools import setup

package_name = 'forest_core'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # you have forest_core/__init__.py
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/avoid_lidar.launch.py',
            'launch/frontier_explorer.launch.py',
            'launch/both.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saket',
    maintainer_email='Saket.V.Nadimpalli@student.uts.edu.au',
    description='Core nodes and launches for the forest sim',
    license='MIT',
    entry_points={
        'console_scripts': [
            'avoid_lidar = forest_core.avoid_lidar:main',
            'frontier_explorer = forest_core.frontier_explorer:main',
        ],
    },
)
