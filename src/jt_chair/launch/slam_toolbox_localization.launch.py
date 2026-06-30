import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    jt_chair_share = get_package_share_directory('jt_chair')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    
    map_file = "/home/orangepi/slam_ws/src/jt_chair/map/105_2"


    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(jt_chair_share, 'config', 'mapper_params_localization.yaml')
    )

    rviz_config_file = os.path.join(jt_chair_share, 'rviz', 'cartographer.rviz')

    # 1) 静态 TF: base_link -> laser
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        output='screen',
        arguments=['0.40', '-0.25', '0.24', '0.0', '0.0', '0.0', 'base_link', 'laser'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 2) 激光雷达（LSN10）
    laser_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lslidar_driver'),
                'launch',
                'lsn10_launch.py'
            ])
        )
    )

    # 3) 里程计节点（如果你要用 use_odometry=true，必须保证它不挂且发布 odom / TF）
    odom_node = Node(
        package='dsw_chair',
        executable='dsw_chair',
        name='odom_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 4) slam_toolbox 定位节点
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
            {'map_file_name': map_file},   # 指向 posegraph 文件
        ]
       
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Open RViz for visualization'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(jt_chair_share, 'config', 'mapper_params_localization.yaml'),
            description='Path to SLAM Toolbox localization parameter file'
        ),

        laser_node,
        odom_node,
        static_tf_node,
        slam_toolbox_node,
        rviz_node,
    ])
