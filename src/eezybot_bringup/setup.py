from glob import glob

from setuptools import find_packages, setup

package_name = 'eezybot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),   # gồm cả eezybot_bringup.core (các file lõi copy nguyên)
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vdlong',
    maintainer_email='user@example.com',
    description='EEZYbotARM MK2 trong RViz2: thanh truot goc servo, chu trinh gap - tha co ne vat can, tay gia lap hoac tay that.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'slider_node = eezybot_bringup.slider_node:main',
            'sim_node = eezybot_bringup.sim_node:main',
            'fake_esp32 = eezybot_bringup.fake_esp32:main',
        ],
    },
)
