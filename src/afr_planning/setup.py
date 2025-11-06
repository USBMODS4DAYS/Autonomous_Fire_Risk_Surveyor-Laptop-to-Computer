from setuptools import setup

package_name = 'afr_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/bringup.launch.py',
            'launch/frontier_explorer.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/twist_mux.yaml']),
    ],


    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saket',
    maintainer_email='saket@todo.todo',
    description='Custom A* planner and path follower',
    license='BSD-2-Clause',
    entry_points={
        'console_scripts': [
            'astar_planner = afr_planning.astar_planner:main',
            'path_follower = afr_planning.path_follower:main',
            'frontier_explorer = afr_planning.frontier_explorer:main',

        ],
    },
)
