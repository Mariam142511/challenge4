import os
from glob import glob
from setuptools import setup

package_name = 'challenge_04'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['params.yaml']),  
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='puzzlebot',
    maintainer_email='puzzlebot@todo.todo',
    description='Package for Half Term Challenge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller10 = challenge_04.controller10:main',
            'odometry = challenge_04.odometry:main',
            'path3 = challenge_04.path3:main',
            'pathCustom = challenge_04.pathCustom:main',
            'cameraPub = challenge_04.cameraPub:main',
            'susImage4 = challenge_04.susImage4:main',
            'camPath4 = challenge_04.camPath4:main',
        ],
    },
)
