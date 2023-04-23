from setuptools import setup

package_name = 'mi_robot_manipulador_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotica',
    maintainer_email='robotica@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_manipulador_teleop = mi_robot_manipulador_2.robot_manipulador_teleop:main',
            'robot_manipulador_teleop_test = mi_robot_manipulador_2.robot_manipulador_teleop:main'
        ],
    },
)
