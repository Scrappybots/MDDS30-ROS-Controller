from setuptools import find_packages, setup

package_name = 'mdds30_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Bryan Ribas',
    maintainer_email='bryanribas@gmail.com',
    description='Interface with the Cytron SmartDriveDuo-30 MDDS30 motor controller',
    license='CC BY-SA 4.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mdds30_controller = mdds30_controller.mdds30_controller:main',
        ],
    },
)
