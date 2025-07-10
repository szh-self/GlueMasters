from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'gluemaster'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+"/launch", glob('launch/*.launch.py')),
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gm_machine_node = gluemaster.GlueMasterMachine_Node:main',
            'gm_ui_node = gluemaster.ui:main',
            'gm_encoder_node = gluemaster.RE_node:main',
            'gm_rec_node = gluemaster.Rec_Node:main',
        ],
    },
)
