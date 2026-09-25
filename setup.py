from setuptools import find_packages, setup

package_name = 'panda_pick_place'

setup(
    name=package_name,
    version='0.0.1',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='Sajjad Hussain',
    maintainer_email='sh1755@example.com',

    description='ROS 2 MoveIt 2 Panda pick-and-place simulation',

    license='Apache-2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'pick_place = panda_pick_place.pick_place:main',
            'add_scene = panda_pick_place.add_scene:main',
        ],
    },
)
