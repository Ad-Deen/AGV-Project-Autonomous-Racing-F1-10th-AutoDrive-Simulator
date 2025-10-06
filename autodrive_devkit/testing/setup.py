from setuptools import setup

package_name = 'racing_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/map_stack.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'throttle_publisher = racing_agent.throttle_publisher:main',
            'localization = racing_agent.localization:main',
            'mapping = racing_agent.mapping:main',
            'map_process = racing_agent.map_process:main',
        ],
    },
)
