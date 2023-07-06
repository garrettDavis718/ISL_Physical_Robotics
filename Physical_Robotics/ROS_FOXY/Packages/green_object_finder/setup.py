from setuptools import setup

package_name = 'green_object_finder'
submodules = 'green_object_finder/submodules'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leviticuslintag',
    maintainer_email='leviticuslintag@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'closest_objects = green_object_finder.closest_objects:main',
            'turn_to_objects = green_object_finder.turn_to_objects:main' 
        ],
    },
)
