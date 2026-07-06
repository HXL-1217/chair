# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.conditions import IfCondition
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.actions import IncludeLaunchDescription


# def generate_launch_description():
#     jt_chair_share = get_package_share_directory('jt_chair')

#     use_sim_time = LaunchConfiguration('use_sim_time')
#     use_rviz = LaunchConfiguration('use_rviz')

#     map_file = "/home/orangepi/slam_ws/src/jt_chair/map/zh"  # 指向你的 posegraph 文件，注意路径和文件名

#     ekf_config_file = os.path.join(jt_chair_share, 'config', 'ekf.yaml')

#     slam_params_file = LaunchConfiguration(
#         'slam_params_file',
#         default=os.path.join(jt_chair_share, 'config', 'double_localization.yaml')
#     )

#     rviz_config_file = os.path.join(jt_chair_share, 'rviz', 'cartographer.rviz')

#     # 1) 静态 TF: base_link -> laser
#     static_tf_node_1 = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='static_tf_publisher_1',
#         arguments=['0.29', '-0.255', '0.2', '0.0', '0.0', '0.0', 'base_link', 'laser_1']
#     )

#     static_tf_node_2 = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='static_tf_publisher_2',
#         arguments=['-0.29', '0.255', '0.2', '3.124', '0.0', '0.0', 'base_link', 'laser_2']
#     )

#     # 2) 激光雷达（LSN10）
#     laser_node = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([
#                 FindPackageShare('lslidar_driver'),
#                 'launch',
#                 'lsn10p_double_launch.py'
#             ])
#         )
#     )

#     # 5) 激光数据融合节点 (将 /scan_1 和 /scan_2 融合为 /scan_merged)
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

#     # 3) 里程计节点（如果你要用 use_odometry=true，必须保证它不挂且发布 odom / TF）
#     odom_node = Node(
#         package='dsw_chair',
#         executable='dsw_chair',
#         name='odom_publisher',
#         output='screen',
#         parameters=[{'use_sim_time': use_sim_time}]
#     )

#     # 4) IMU 的静态 TF 坐标发布
#     static_tf_imu = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='static_tf_publisher_imu',
#         arguments=['0.185', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
#     )

#     # 5) fdilink_ahrs IMU 驱动
#     imu_driver_node = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([
#                 FindPackageShare('fdilink_ahrs'),
#                 'launch',
#                 'ahrs_driver.launch.py'
#             ])
#         )
#     )

#     # 6) EKF (robot_localization) 融合节点
#     ekf_node = Node(
#         package='robot_localization',
#         executable='ekf_node',
#         name='ekf_filter_node',
#         output='screen',
#         parameters=[
#             ekf_config_file,
#             {'use_sim_time': use_sim_time}
#         ],
#         remappings=[('/imu/data', '/imu')]
#     )

#     # 7) slam_toolbox 定位节点
#     slam_toolbox_node = Node(
#         package='slam_toolbox',
#         executable='localization_slam_toolbox_node',
#         name='slam_toolbox',
#         output='screen',
#         parameters=[
#             slam_params_file,
#             {'use_sim_time': use_sim_time},
#             {'map_file_name': map_file},   # 指向 posegraph 文件
#         ]
#     )

#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         arguments=['-d', rviz_config_file],
#         parameters=[{'use_sim_time': use_sim_time}],
#         output='screen',
#         condition=IfCondition(use_rviz)
#     )

#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='false',
#             description='Use simulation (Gazebo) clock if true'
#         ),
#         DeclareLaunchArgument(
#             'use_rviz',
#             default_value='false',
#             description='Open RViz for visualization'
#         ),
#         DeclareLaunchArgument(
#             'slam_params_file',
#             default_value=os.path.join(jt_chair_share, 'config', 'double_localization.yaml'),
#             description='Path to SLAM Toolbox localization parameter file'
#         ),

#         laser_node,
#         static_tf_node_1,
#         static_tf_node_2,
#         static_tf_imu,
#         imu_driver_node,
#         odom_node,
#         ekf_node,
#         scan_merger_node,
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
    jt_chair_share = get_package_share_directory('jt_chair')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    map_file = "/home/orangepi/slam_ws/src/jt_chair/map/ysg1"  # 指向你的 posegraph 文件，注意路径和文件名

    ekf_config_file = os.path.join(jt_chair_share, 'config', 'ekf.yaml')

    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(jt_chair_share, 'config', 'double_localization.yaml')
    )

    rviz_config_file = os.path.join(jt_chair_share, 'rviz', 'cartographer.rviz')

    # 1) 静态 TF: base_link -> laser
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

    # 2) 激光雷达（LSN10）
    laser_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lslidar_driver'),
                'launch',
                'lsn10p_double_launch.py'
            ])
        )
    )

    # 3) 激光数据融合节点 (将 /scan_1 和 /scan_2 融合为 /scan_merged)
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

    # 4) 里程计节点
    odom_node = Node(
        package='dsw_chair',
        executable='dsw_chair',
        name='odom_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 5) IMU 的静态 TF 坐标发布
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher_imu',
        arguments=['0.185', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
    )

    # 6) fdilink_ahrs IMU 驱动
    imu_driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fdilink_ahrs'),
                'launch',
                'ahrs_driver.launch.py'
            ])
        )
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
        remappings=[('/imu/data', '/imu')]
    )

    # 8) slam_toolbox 定位节点
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
            {'map_file_name': map_file},
        ]
    )

    # ------------------------
    # [新增] 9) 语音桥接节点 (纯挂载)
    # ------------------------
    voice_nav_node = Node(
        package='jt_chair',
        executable='voice_nav_bridge',
        name='voice_nav_bridge',
        output='screen'
    )

    # 10) RViz 节点
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
            default_value=os.path.join(jt_chair_share, 'config', 'double_localization.yaml'),
            description='Path to SLAM Toolbox localization parameter file'
        ),

        laser_node,
        static_tf_node_1,
        static_tf_node_2,
        static_tf_imu,
        imu_driver_node,
        odom_node,
        ekf_node,
        scan_merger_node,
        slam_toolbox_node,
        
        # 仅加入语音节点
        voice_nav_node,
        
        rviz_node,
    ])