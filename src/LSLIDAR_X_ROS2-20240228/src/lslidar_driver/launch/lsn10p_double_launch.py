#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os

def generate_launch_description():

    driver_dir_1 = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lidar_uart_ros2', 'lsn10p_1.yaml')
    driver_dir_2 = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lidar_uart_ros2', 'lsn10p_2.yaml')

    driver_node_1 = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node1',  # 第一个雷达节点名
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[driver_dir_1],
    )

    driver_node_2 = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node2',  # 第二个雷达节点名
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[driver_dir_2],
    )

    return LaunchDescription([
        driver_node_1,
        driver_node_2,
    ])

# #!/usr/bin/python3
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import LifecycleNode
# from launch_ros.actions import Node
# import os

# def generate_launch_description():

#     driver_dir_1 = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lidar_uart_ros2', 'lsn10p_1.yaml')
#     driver_dir_2 = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lidar_uart_ros2', 'lsn10p_2.yaml')

#     driver_node_1 = LifecycleNode(
#         package='lslidar_driver',
#         executable='lslidar_driver_node',
#         name='lslidar_driver_node1',  # 必须与 yaml 中的顶层名字一致
#         output='screen',
#         emulate_tty=True,
#         namespace='',
#         parameters=[driver_dir_1],
#     )

#     driver_node_2 = LifecycleNode(
#         package='lslidar_driver',
#         executable='lslidar_driver_node',
#         name='lslidar_driver_node2',  # 必须与 yaml 中的顶层名字一致
#         output='screen',
#         emulate_tty=True,
#         namespace='',
#         parameters=[driver_dir_2],
#     )

#     # arguments 顺序: x, y, z, yaw, pitch, roll, frame_id, child_frame_id
#     static_tf_node_1 = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='static_tf_publisher_1',
#         arguments=['0.31', '-0.305', '0.2', '0.0', '0.0', '0.0', 'base_link', 'laser_1']
#     )

#     static_tf_node_2 = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='static_tf_publisher_2',
#         arguments=['-0.31', '0.285', '0.2', '3.1415926', '0.0', '0.0', 'base_link', 'laser_2']
#     )

#     return LaunchDescription([
#         driver_node_1,
#         driver_node_2,
#         static_tf_node_1,
#         static_tf_node_2,
#     ])