from setuptools import find_packages, setup

package_name = 'my_subscriber_pkg'

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
    maintainer='juan',
    maintainer_email='juan_p.rivera@uao.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'subscriber_node = my_subscriber_pkg.subscriber_node:main',  # Entry point for your subscriber node
             'teleop_node = my_subscriber_pkg.teleop_node:main',
             'my_teleop_node = my_subscriber_pkg.my_teleop_node:main',
             'my2_teleop_node = my_subscriber_pkg.my2_teleop_node:main',
             'gui_node = my_subscriber_pkg.gui_node:main',
        ],
    },
)
