import os
from glob import glob
from setuptools import setup

package_name = 'usv_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.*')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    
    # =================================================================
    # ============== 【 这 里 是 核 心 修 改 】 ==========================
    # =================================================================
    entry_points={
        'console_scripts': [
            'usv_controller = usv_navigation.usv_controller:main',
            
            # 保留您其他的脚本
            'initialpose_bridge = usv_navigation.initialpose_to_service_bridge:main',
            'manual_tuner = usv_navigation.manual_tuner:main',
        ],
    },
)