from glob import glob
import os


from setuptools import find_packages, setup

package_name = 'indoor_bot_evaluation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Avantika Ajit',
    maintainer_email='avantika.aji22@gmail.com',
    description='Evaluation and metrics tooling (e.g. Nav2 ON/OFF ablation) for indoor_bot.',
    license='AGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
