from setuptools import find_packages, setup

package_name = 'go_green'

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
    maintainer='rschnurr',
    maintainer_email='rschnurr123@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'offboard_control = go_green.offboard_control:main',
            'go_green = go_green.go_green:main',
            'go_green_autoland = go_green.go_green_autoland:main',
            'green_tracker = go_green.green_tracker:main',
        ],
    },
)
