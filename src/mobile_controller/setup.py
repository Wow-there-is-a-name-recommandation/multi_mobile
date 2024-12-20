from setuptools import find_packages, setup

package_name = 'mobile_controller'

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
    maintainer='jo',
    maintainer_email='jo000320@kw.ac.kr',
    description='Creat velocity control for Multi mobile',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'consensus_controller = mobile_controller.consensus_controller:main',
            'state_communicate = mobile_controller.state_communicate:main',
            'laplacian_algorithm = mobile_controller.laplacian_algorithm:main',
        ],
    },
)
