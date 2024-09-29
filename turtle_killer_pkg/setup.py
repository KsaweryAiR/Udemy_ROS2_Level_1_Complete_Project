from setuptools import find_packages, setup

package_name = 'turtle_killer_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ksawery',
    maintainer_email='85024534+KsaweryAiR@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtle_spawner = turtle_killer_pkg.turtle_spawner:main",
            "turtle_controller = turtle_killer_pkg.turtle_controller:main",
            "turtle_controller_v2 = turtle_killer_pkg.turtle_controller_v2:main",
            "turtle_controller_v3 = turtle_killer_pkg.turtle_controller_v3:main",
        ],
    },
)
