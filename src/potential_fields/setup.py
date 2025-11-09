from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'potential_fields'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pedro',
    maintainer_email='pedro@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'potential_field = potential_fields.potential_field_navigator:main',
        ],
    },
)
