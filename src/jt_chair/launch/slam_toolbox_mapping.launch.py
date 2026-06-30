import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.actions import LogInfo


def generate_launch_description():
    # 包路径
    jt_chair_share = get_package_share_directory('jt_chair')

    # Launch 参数（由 DeclareLaunchArgument 赋初值）
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    slam_params_file = LaunchConfiguration('slam_params_file')
    

    # RViz 配置
    rviz_config_file = os.path.join(jt_chair_share, 'rviz', 'slam_toolbox.rviz')

    # ====== 节点定义 ======

    # 1) 静态 TF: base_link → laser
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        output='screen',
        arguments=['0.40', '-0.25', '0.24', '0.0', '0.0', '0.0', 'base_link', 'laser'],
        # arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'laser'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 2) 激光雷达 (LSN10)
    laser_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lslidar_driver'),
                'launch',
                'lsn10_launch.py'
            ])
        )
    )

    # 3) 里程计
    odom_node = Node(
        package='dsw_chair',
        executable='dsw_chair',
        name='odom_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 4) SLAM Toolbox 建图节点（online_sync 模式）
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('scan', '/scan'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # 5) RViz（可选）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    # ====== 返回 LaunchDescription ======
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Open RViz for visualization'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(jt_chair_share, 'config', 'mapper_params_online_sync.yaml'),
            description='SLAM Toolbox mapping parameters'
        ),

        # 启动顺序
        # LogInfo(msg=['当前 use_sim_time 的值为: ', use_sim_time]),
        laser_node,
        odom_node,
        static_tf_node,
        slam_toolbox_node,
        rviz_node,
    
    ])

