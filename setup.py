from setuptools import find_packages, setup

package_name = 'coral_move'

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
    maintainer='Ian Kim',
    maintainer_email='KimI27@student.jhs.net',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coral_move = coral_move.coral_move:main'
            'auto_movement = auto_movement.auto_movement:main'
        ],
    },
)
