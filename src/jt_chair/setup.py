# from setuptools import setup
# import os
# from glob import glob

# package_name = 'jt_chair'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=[],
#     data_files=[
#         # 安装 package.xml 等元数据
#         ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),

#         # 安装 launch 文件（支持多个 .launch.py）
#         (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

#         # 安装 config 文件（.lua, .yaml 等）
#         (os.path.join('share', package_name, 'config'), glob('config/*')),

#         # 安装 rviz 配置文件
#         (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
#         # 安装 map
#         (os.path.join('share', package_name, 'map'), glob('map/*.yaml') + glob('map/*.pgm')),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='Your Name',
#     maintainer_email='your_email@example.com',
#     description='Launch and config files for jt_chair robot',
#     license='Apache-2.0',
#     tests_require=['pytest'],
# )
from setuptools import setup
import os
from glob import glob

package_name = 'jt_chair'

setup(
    name=package_name,
    version='0.0.0',
    # [修改点 1] 这里告诉系统去加载同名的 Python 模块文件夹
    packages=[package_name], 
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'map'), glob('map/*.yaml') + glob('map/*.pgm')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Launch and config files for jt_chair robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    
    # [修改点 2] 这里是注册节点的灵魂！
    # 它的意思是：在 jt_chair 文件夹下的 voice_nav_bridge.py 里找 main 函数，
    # 把它编译成一个叫 voice_nav_bridge 的可执行节点。
    entry_points={
        'console_scripts': [
            'voice_nav_bridge = jt_chair.voice_nav_bridge:main'
        ],
    },


#     entry_points={
#     'console_scripts': [
#         'voice_nav_bridge = jt_chair.voice_nav_core:main',
#     ],
# },
)