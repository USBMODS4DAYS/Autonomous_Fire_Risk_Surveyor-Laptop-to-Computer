# from setuptools import setup

# package_name = 'path_smoother'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=[package_name],
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='your_name',
#     maintainer_email='your_email@example.com',
#     description='Path smoothing for Nav2 A* path',
#     license='Apache License 2.0',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             'path_smoother_node = path_smoother.path_smoother_node:main',
#         ],
#     },
# )

# file moi 

# # from setuptools import setup
# # import os
# # from glob import glob

# # package_name = 'path_smoother'

# # # Chỉ cần tìm folder 'config'
# # config_folder = os.path.join('config')

# # setup(
# #     name=package_name,
# #     version='0.0.0',
# #     # Dòng này sẽ tìm thư mục 'path_smoother'
# #     # và tất cả các file .py bên trong nó (bao gồm cả 2 node của bạn)
# #     packages=[package_name], 
# #     data_files=[
# #         ('share/ament_index/resource_index/packages',
# #             ['resource/' + package_name]),
# #         ('share/' + package_name, ['package.xml']),
        
# #         # Cài đặt file config 'my_waypoints.yaml'
# #         (os.path.join('share', package_name, 'config'), glob(os.path.join(config_folder, '*.yaml'))),
# #     ],
# #     install_requires=['setuptools', 'PyYAML'], 
# #     zip_safe=True,
# #     maintainer='your_name',
# #     maintainer_email='your_email@example.com',
# #     description='Path smoothing for Nav2 A* path',
# #     license='Apache License 2.0',
# #     tests_require=['pytest'],
# #     entry_points={
# #         'console_scripts': [
# #             # Dòng 1: Node cũ của bạn
# #             'path_smoother_node = path_smoother.path_smoother_node:main',
            
# #             # Dòng 2: Node mới (đã bỏ 'scripts.')
# #             'waypoint_runner = path_smoother.waypoint_runner:main',
# #         ],
# #     },
# # )

# file moi 2 

from setuptools import setup
import os
from glob import glob

package_name = 'path_smoother'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Path smoothing for Nav2 A* path and waypoint navigation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Node làm mượt đường đi Nav2
            'path_smoother_node = path_smoother.path_smoother_node:main',
            
            # Node điều hướng waypoints
            'waypoint_navigator = path_smoother.waypoint_navigator:main',
        ],
    },
)