from setuptools import setup

package_name = 'hybrid_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rens Huberts',
    maintainer_email='rp.huberts@student.han.nl',
    description='A hybrid package with both C++ and Python support',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'python_node = hybrid_pkg.python_node:main',
        ],
    },
)
