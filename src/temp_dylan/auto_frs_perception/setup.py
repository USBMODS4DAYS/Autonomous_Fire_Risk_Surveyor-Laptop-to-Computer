from setuptools import find_packages, setup

package_name = 'auto_frs_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    # Install package.xml and resource index
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),

    # Install models into share/<pkgname>/models
    ('share/' + package_name + '/models', [
        'models/tm_model.h5',
        'models/tm_labels.txt',
    ]),
],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'hotspot_detector = auto_frs_perception.hotspot_detector:main',
        'test_sub = auto_frs_perception.test_sub:main',
        'object_mapper = auto_frs_perception.object_mapper:main',
    ],
},

)
