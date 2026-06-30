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
        default=os.path.join(jt_chair_share, 'config', 'mapper_params_localization.yaml')
    )

    map_file = "/home/orangepi/slam_ws/src/jt_chair/map/new1"       

    # nav2 params
    nav2_params_file = LaunchConfiguration(
        'nav2_params_file',
        default=os.path.join(jt_chair_share, 'config', 'nav2_params_wheelchair.yaml')
    )

    # RViz config (建议用 nav2 自带的或你自己配的)
    rviz_config_file = LaunchConfiguration(
        'rviz_config',
        default=os.path.join(jt_chair_share, 'rviz', 'slam_toolbox.rviz')
    )

    # ------------------------
    # Nodes: Sensors & TF
    # ------------------------
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        output='screen',
        # x y z roll pitch yaw frame_id child_frame_id
        arguments=['0.40', '-0.25', '0.24', '0.0', '0.0', '0.0', 'base_link', 'laser'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    laser_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lslidar_driver'),
                'launch',
                'lsn10_launch.py'
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
            default_value=os.path.join(jt_chair_share, 'config', 'mapper_params_localization.yaml'),
            description='slam_toolbox localization params yaml'
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(jt_chair_share, 'config', 'nav2_params_wheelchair.yaml'),
            description='Nav2 params yaml'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(jt_chair_share, 'rviz', 'nav2.rviz'),
            description='RViz config file'
        ),

        laser_node,
        odom_node,
        static_tf_node,
        slam_toolbox_node,
        nav2_navigation,
        rviz_node,
    ])
