from setuptools import setup

package_name = 'uts_forest_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],   # no need for find_packages, simpler here
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saket',
    maintainer_email='Saket.V.Nadimpalli@student.uts.edu.au',
    description='Wrapper bringup for UTS forest sim (sets model path and spawns UAV)',
    license='MIT',
)
