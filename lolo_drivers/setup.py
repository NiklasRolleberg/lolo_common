from setuptools import find_packages, setup
import glob, os

package_name = 'lolo_drivers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niklas',
    maintainer_email='nrol@kth.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'menu_input = lolo_drivers.menu_input:main',
            'menu_output = lolo_drivers.menu_output:main',
            'usbl_interface = lolo_drivers.usbl_interface:main',
            'geofence_checker_client = lolo_drivers.geofence_checker_client:main',
        ],
    },
)
