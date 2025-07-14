from setuptools import find_packages, setup

package_name = 'task_5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/red_ball_tracker_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anirudh',
    maintainer_email='anirudh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'red_ball_tracker = task_5.red_ball_tracker:main',
        ],
    },
)
