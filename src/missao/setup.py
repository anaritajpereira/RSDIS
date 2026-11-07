from glob import glob

from setuptools import setup

package_name = 'missao'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arjp',
    maintainer_email='anarita.j.pereira@gmail.com',
    description='Busca e salvamento marítimo multi-robô (drone + barco robô).',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'poses = missao.poses:main',
            'drone_missao = missao.drone_missao:main',
            'barco_missao = missao.barco_missao:main',
            'detetor = missao.detetor:main',
        ],
    },
)
