# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     jt_chair_share = get_package_share_directory('jt_chair')

#     # ------------------------
#     # Launch args
#     # ------------------------
#     use_sim_time = LaunchConfiguration('use_sim_time', default='false')
#     use_rviz = LaunchConfiguration('use_rviz', default='false')

#     # slam_toolbox localization params
#     slam_params_file = LaunchConfiguration(
#         'slam_params_file',
#         default=os.path.join(jt_chair_share, 'config', 'double_localization.yaml')
#     )

#     map_file = "/home/orangepi/slam_ws/src/jt_chair/map/tz_1"       

#     # nav2 params
#     nav2_params_file = LaunchConfiguration(
#         'nav2_params_file',
#         default=os.path.join(jt_chair_share, 'config', 'nav2_params_dw.yaml')
#     )

#     # RViz config
#     rviz_config_file = LaunchConfiguration(
#         'rviz_config',
#         default=os.path.join(jt_chair_share, 'rviz', 'slam_toolbox.rviz')
#     )

#     # [新增] EKF 配置文件路径
#     ekf_config_file = os.path.join(jt_chair_share, 'config', 'ekf.yaml')

#     # ------------------------
#     # Nodes: Sensors & TF
#     # ------------------------
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

#     # [新增] IMU 的静态 TF 坐标发布
#     static_tf_imu = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='static_tf_publisher_imu',
#         arguments=['0.0', '0.0', '0.2', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
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

#     # [新增] 引入 fdilink_ahrs IMU 驱动
#     imu_driver_node = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([
#                 FindPackageShare('fdilink_ahrs'),
#                 'launch',
#                 'ahrs_driver.launch.py'
#             ])
#         )
#     )

#     odom_node = Node(
#         package='dsw_chair',
#         executable='dsw_chair',
#         name='odom_publisher',
#         output='screen',
#         parameters=[{'use_sim_time': use_sim_time}],
#     )

#     # [新增] EKF (robot_localization) 融合节点
#     ekf_node = Node(
#         package='robot_localization',
#         executable='ekf_node',
#         name='ekf_filter_node',
#         output='screen',
#         parameters=[
#             ekf_config_file,
#             {'use_sim_time': use_sim_time}
#         ],
#         # 如果你的 ekf.yaml 订阅 /imu/data，但驱动发布 /imu，取消以下注释
#         remappings=[('/imu/data', '/imu')] 
#     )

#     scan_merger_node = Node(
#         package='dual_laser_merger',
#         executable='dual_laser_merger_node', 
#         name='dual_laser_merger',
#         output='screen',
#         parameters=[{
#             'laser_1_topic': '/scan_1',      
#             'laser_2_topic': '/scan_2',      
#             'merged_scan_topic': '/scan_merged',  
#             'target_frame': 'base_link',     
#             'publish_rate': 20,              
#             'angle_increment': 0.00698,       
#             'scan_time': 0.1,
#             'range_min': 0.20,
#             'range_max': 25.0,
#             'use_inf': False                 
#         }]
#     )

#     # ------------------------
#     # slam_toolbox Localization
#     # ------------------------
#     slam_toolbox_node = Node(
#         package='slam_toolbox',
#         executable='localization_slam_toolbox_node',
#         name='slam_toolbox',
#         output='screen',
#         parameters=[
#             slam_params_file,
#             {'use_sim_time': use_sim_time},
#             {'map_file_name': map_file},  
#         ],
#         # [补充优化] 建议将 TF 重映射加上，与建图保持一致
#         remappings=[
#             ('/tf', 'tf'),
#             ('/tf_static', 'tf_static'),
#         ],
#     )

#     # ------------------------
#     # Nav2 Navigation (NO amcl/map_server)
#     # ------------------------
#     nav2_navigation = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([
#                 FindPackageShare('nav2_bringup'),
#                 'launch',
#                 'navigation_launch.py'
#             ])
#         ),
#         launch_arguments={
#             'use_sim_time': use_sim_time,
#             'autostart': TextSubstitution(text='true'),
#             'params_file': nav2_params_file,
#             # 嵌入式上建议先关组合模式，省内存更稳
#             'use_composition': TextSubstitution(text='False'),
#         }.items()
#     )

#     # ------------------------
#     # RViz (optional)
#     # ------------------------
#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         arguments=['-d', rviz_config_file],
#         parameters=[{'use_sim_time': use_sim_time}],
#         output='screen',
#         condition=IfCondition(use_rviz)
#     )

#     # ------------------------
#     # LaunchDescription
#     # ------------------------
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
#             description='slam_toolbox localization params yaml'
#         ),
#         DeclareLaunchArgument(
#             'nav2_params_file',
#             default_value=os.path.join(jt_chair_share, 'config', 'nav2_params_dw.yaml'),
#             description='Nav2 params yaml'
#         ),
#         DeclareLaunchArgument(
#             'rviz_config',
#             default_value=os.path.join(jt_chair_share, 'rviz', 'nav2.rviz'),
#             description='RViz config file'
#         ),

#         laser_node,
#         imu_driver_node,      # [新增]
#         static_tf_node_1,
#         static_tf_node_2,
#         static_tf_imu,        # [新增]
#         odom_node,
#         ekf_node,             # [新增]
#         scan_merger_node,
#         slam_toolbox_node,
#         nav2_navigation,
#         rviz_node,
#     ])


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    jt_chair_share = get_package_share_directory('jt_chair')

    # ------------------------
    # Launch args
    # ------------------------
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    # slam_toolbox localization params
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(jt_chair_share, 'config', 'double_localization.yaml')
    )

    map_file = "/home/orangepi/slam_ws/src/jt_chair/map/test_map0814"       
    

    # nav2 params
    nav2_params_file = LaunchConfiguration(
        'nav2_params_file',
        default=os.path.join(jt_chair_share, 'config', 'nav2_params_dw.yaml')
    )

    # RViz config
    rviz_config_file = LaunchConfiguration(
        'rviz_config',
        default=os.path.join(jt_chair_share, 'rviz', 'slam_toolbox.rviz')
    )

    # [新增] EKF 配置文件路径
    ekf_config_file = os.path.join(jt_chair_share, 'config', 'ekf.yaml')

    # ------------------------
    # Nodes: Sensors & TF
    # ------------------------
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

    # [新增] IMU 的静态 TF 坐标发布
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher_imu',
        arguments=['0.185', '0.0', '0.2', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
    )

    laser_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lslidar_driver'),
                'launch',
                'lsn10p_double_launch.py'
            ])
        )
    )

    # [新增] 引入 fdilink_ahrs IMU 驱动
    imu_driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fdilink_ahrs'),
                'launch',
                'ahrs_driver.launch.py'
            ])
        )
    )

    odom_node = Node(
        package='dsw_chair',
        executable='dsw_chair',
        name='odom_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # [新增] EKF (robot_localization) 融合节点
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_file,
            {'use_sim_time': use_sim_time}
        ],
        # 如果你的 ekf.yaml 订阅 /imu/data，但驱动发布 /imu，取消以下注释
        remappings=[('/imu/data', '/imu')] 
    )

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

    # ------------------------
    # slam_toolbox Localization
    # ------------------------
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
            {'map_file_name': map_file},  
        ],
        # [补充优化] 建议将 TF 重映射加上，与建图保持一致
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # ------------------------
    # Nav2 Navigation (NO amcl/map_server)
    # ------------------------
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': TextSubstitution(text='true'),
            'params_file': nav2_params_file,
            # 嵌入式上建议先关组合模式，省内存更稳
            'use_composition': TextSubstitution(text='False'),
        }.items()
    )

    # ------------------------
    # Voice Navigation Bridge
    # ------------------------
    # [新增] 语音导航桥接节点
    voice_nav_node = Node(
        package='jt_chair',             # 假设你的 python 文件建在这个包里，若不是请修改
        executable='voice_nav_bridge',  # 这里填入 setup.py / CMakeLists 中注册的可执行文件名称
        name='voice_nav_bridge',
        output='screen'
    )

    # ------------------------
    # RViz (optional)
    # ------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    # ------------------------
    # LaunchDescription
    # ------------------------
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
            description='slam_toolbox localization params yaml'
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(jt_chair_share, 'config', 'nav2_params_dw.yaml'),
            description='Nav2 params yaml'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(jt_chair_share, 'rviz', 'nav2.rviz'),
            description='RViz config file'
        ),

        laser_node,
        imu_driver_node,      # [新增]
        static_tf_node_1,
        static_tf_node_2,
        static_tf_imu,        # [新增]
        odom_node,
        ekf_node,             # [新增]
        scan_merger_node,
        slam_toolbox_node,
        nav2_navigation,
        voice_nav_node,       # [新增] 语音解析和目标下发节点
        rviz_node,
    ])