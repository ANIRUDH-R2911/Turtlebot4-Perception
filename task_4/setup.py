from setuptools import find_packages, setup

package_name = 'task_4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/object_detector_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anirudh',
    maintainer_email='anirudh@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = task_4.image_publisher:main', 
            'object_detector = task_4.object_detector:main',
        ],
    },
)
