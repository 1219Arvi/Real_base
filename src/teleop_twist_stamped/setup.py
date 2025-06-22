from setuptools import find_packages, setup

package_name = 'teleop_twist_stamped'

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
    maintainer='arvi',
    maintainer_email='thapliyal1219@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'teleop_twist_stamped_keyboard = teleop_twist_stamped.teleop_twist_stamped_keyboard:main',
        ],
},
)
