from setuptools import find_packages, setup

package_name = 'uart_bridge_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='UART bridge node for STM32 and Orange Pi 3b',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uart_bridge_node = uart_bridge_pkg.uart_bridge_node:main'
        ],
    },
)
