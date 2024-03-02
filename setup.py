from setuptools import find_packages, setup

package_name = 'autocoral'

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
    maintainer='jhsrobo',
    maintainer_email='eandid27@student.jhs.net',
    description='Autonomous movement for brain coral task',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coral_move = autocoral.coral_move:main',
            'target_detection = autocoral.target_detection:main'
        ],
    },
)
