from setuptools import setup

package_name = 'shipsim_sensor_module'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mitsuyuki',
    maintainer_email='mitsuyuki-taiga-my@ynu.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "simulated_kt_sensor_node = shipsim_sensor_module.simulated_kt_sensor_node:main",
            "simulated_mmg_sensor_node = shipsim_sensor_module.simulated_mmg_sensor_node:main"
        ],
    },
)
