from setuptools import find_packages, setup

package_name = 'control_room'

setup(
    name=package_name,
    version='0.0.0',
    packages=['utils'],
    package_dir={'': 'include'},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.xml']),
        ('lib/' + package_name, ['control_room/main.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tatyana',
    maintainer_email='tatiana.berlenko@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            "control_room = control_room.main:main",
        ],
    },
)
