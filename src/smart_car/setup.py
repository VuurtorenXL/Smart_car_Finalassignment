from setuptools import setup

package_name = 'smart_car'  # Replace with your package name

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rens Huberts',
    maintainer_email='rp.huberts@student.han.nl',
    description='A hybrid package with both C++ and Python nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'keyboard_control = smart_car.scripts.keyboard_control:main',
        'vehicle_status_publisher = smart_car.scripts.vehicle_status:main',
        'odom = smart_car.scripts.odom:main'

    ],
},

)
