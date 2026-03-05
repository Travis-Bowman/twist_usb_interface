from setuptools import find_packages, setup

package_name = 'twist_to_usb'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        ],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='tbowman@ltu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'twist_to_usb = twist_to_usb.twist_to_usb_node:main'
        ],
    },
)
