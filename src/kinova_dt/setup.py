from setuptools import setup
import os
from glob import glob

package_name = 'kinova_dt'

# collect launch and rviz files
launch_files = glob(os.path.join('launch', '*.launch.py'))
rviz_files = glob(os.path.join('rviz', '*.rviz'))

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

if launch_files:
    data_files.append((f'share/{package_name}/launch', launch_files))
if rviz_files:
    data_files.append((f'share/{package_name}/rviz', rviz_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='acis',
    maintainer_email='quezadacoloradodiego@gmail.com',
    description='Kinova DT package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gui_joint_control = kinova_dt.gui_joint_control:main',
            'joint_trajectory_bridge = kinova_dt.joint_trajectory_bridge:main',
            'follow_joint_bridge = kinova_dt.follow_joint_trajectory_bridge:main',
            'dual_joint_control = kinova_dt.dual_joint_control:main',
        ],
    },
)
