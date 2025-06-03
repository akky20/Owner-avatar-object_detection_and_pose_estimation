from setuptools import find_packages, setup
import glob

package_name = 'pointcloud_object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*py')),
        ('share/' + package_name + '/sessions', glob.glob('sessions/*bin')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carslabpc',
    maintainer_email='carslabpc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
