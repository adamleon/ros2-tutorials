from setuptools import find_packages, setup

package_name = 'colcon_build'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/config", ['config/some_params.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adamleon',
    maintainer_email='adam.l.kleppe@ntnu.no',
    description='A demonstration package for the use of colcon build',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'some_node = colcon_build.some_node:main'
        ],
    },
)
