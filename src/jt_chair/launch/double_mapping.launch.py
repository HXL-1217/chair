# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.conditions import IfCondition
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# # from launch.actions import LogInfo


# def generate_launch_description():
#     # 包路径
#     jt_chair_share = get_package_share_directory('jt_chair')

#     # Launch 参数（由 DeclareLaunchArgument 赋初值）
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     use_rviz = LaunchConfiguration('use_rviz')
#     slam_params_file = LaunchConfiguration('slam_params_file')
    

#     # RViz 配置
#     rviz_config_file = os.path.join(jt_chair_share, 'rviz', 'slam_toolbox.rviz')


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

#     laser_node = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([
#                 FindPackageShare('lslidar_driver'),
#                 'launch',
#                 'lsn10p_double_launch.py'
#             ])
#         )
#     )

#     # 4) 激光数据融合节点 (将 /scan_1 和 /scan_2 融合为 /scan_merged)
#     scan_merger_node = Node(
#         package='dual_laser_merger',
#         executable='dual_laser_merger_node', # 注意执行文件名称
#         name='dual_laser_merger',
#         output='screen',
#         parameters=[{
#             'laser_1_topic': '/scan_1',      # 雷达 1 的输入话题
#             'laser_2_topic': '/scan_2',      # 雷达 2 的输入话题
#             'merged_scan_topic': '/scan_merged',  # 融合后的输出话题 (给 SLAM 用的)
#             'target_frame': 'base_link',     # 融合的基准坐标系
#             'publish_rate': 20,              # 融合发布频率 (Hz)
#             'angle_increment': 0.00698,       # 匹配你的 LSN10P 雷达参数
#             'scan_time': 0.1,
#             'range_min': 0.20,
#             'range_max': 25.0,
#             'use_inf': False                 # 超出范围的点处理方式
#         }]
#     )

#     # 3) 里程计
#     odom_node = Node(
#         package='dsw_chair',
#         executable='dsw_chair',
#         name='odom_publisher',
#         output='screen',
#         parameters=[{'use_sim_time': use_sim_time}]
#     )

#     # 4) SLAM Toolbox 建图节点（online_sync 模式）
#     slam_toolbox_node = Node(
#         package='slam_toolbox',
#         executable='sync_slam_toolbox_node',
#         name='slam_toolbox',
#         output='screen',
#         parameters=[
#             slam_params_file,
#             {'use_sim_time': use_sim_time}
#         ],
#         remappings=[
#             # ('scan', '/scan_merged'),
#             ('/tf', 'tf'),
#             ('/tf_static', 'tf_static'),
#         ],
#     )

#     # 5) RViz（可选）
#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         arguments=['-d', rviz_config_file],
#         parameters=[{'use_sim_time': use_sim_time}],
#         output='screen',
#         condition=IfCondition(use_rviz)
#     )

#     # ====== 返回 LaunchDescription ======
#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='false',
#             description='Use simulation clock if true'
#         ),
#         DeclareLaunchArgument(
#             'use_rviz',
#             default_value='false',
#             description='Open RViz for visualization'
#         ),
#         DeclareLaunchArgument(
#             'slam_params_file',
#             default_value=os.path.join(jt_chair_share, 'config', 'mapper_params_online_sync.yaml'),
#             description='SLAM Toolbox mapping parameters'
#         ),

#         # 启动顺序
#         # LogInfo(msg=['当前 use_sim_time 的值为: ', use_sim_time]),
#         laser_node,
#         static_tf_node_1,
#         static_tf_node_2,
#         odom_node,
#         scan_merger_node,
#         # static_tf_node,
#         slam_toolbox_node,
#         rviz_node,
    
#     ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 包路径
    jt_chair_share = get_package_share_directory('jt_chair')

    # Launch 参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    # RViz & EKF 配置文件
    rviz_config_file = os.path.join(jt_chair_share, 'rviz', 'slam_toolbox.rviz')
    ekf_config_file = os.path.join(jt_chair_share, 'config', 'ekf.yaml')

    # 1) 两个激光雷达的静态 TF
    static_tf_node_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher_1',
        arguments=['0.29', '-0.255', '0.2', '0.0', '0.0', '0.0', 'base_link', 'laser_1']
    )

    static_tf_node_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher_2',
        arguments=['-0.29', '0.255', '0.2', '3.124', '0.0', '0.0', 'base_link', 'laser_2']
    )

    # 2)  IMU 的静态 TF 坐标发布
    #  frame_id:imu_link
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher_imu',
        arguments=['0.185', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
    )

    # 3) 激光雷达驱动启动
    laser_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lslidar_driver'),
                'launch',
                'lsn10p_double_launch.py'
            ])
        )
    )

    # 4)  fdilink_ahrs 驱动 launch 文件
    imu_driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fdilink_ahrs'),
                'launch',
                'ahrs_driver.launch.py'
            ])
        )
    )

    # 5) 激光数据融合节点
    scan_merger_node = Node(
        package='dual_laser_merger',
        executable='dual_laser_merger_node', 
        name='dual_laser_merger',
        output='screen',
        parameters=[{
            'laser_1_topic': '/scan_1',      
            'laser_2_topic': '/scan_2',      
            'merged_scan_topic': '/scan_merged', 
            'target_frame': 'base_link',     
            'publish_rate': 20,              
            'angle_increment': 0.00698,       
            'scan_time': 0.1,
            'range_min': 0.20,
            'range_max': 25.0,
            'use_inf': False                 
        }]
    )

    # 6) 里程计 (底盘驱动节点，只发 /odom 话题)
    odom_node = Node(
        package='dsw_chair',
        executable='dsw_chair',
        name='odom_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 7) EKF (robot_localization) 融合节点
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/imu/data', '/imu'), 
        ]
    )

    # 8) SLAM Toolbox 节点
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
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # 9) RViz（可选）
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
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Open RViz for visualization'),
        DeclareLaunchArgument('slam_params_file', default_value=os.path.join(jt_chair_share, 'config', 'mapper_params_online_sync.yaml'), description='SLAM Toolbox mapping parameters'),

        # 执行顺序（并无绝对严格要求，但建议先起传感器）
        laser_node,
        imu_driver_node,     # 启动 IMU
        static_tf_node_1,
        static_tf_node_2,
        static_tf_imu,       # 挂载 IMU 的 TF
        odom_node,           # 启动底盘轮子
        ekf_node,            # 启动 EKF 开始融合
        scan_merger_node,
        slam_toolbox_node,
        rviz_node,
    ])