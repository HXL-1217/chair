from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 原始数据录制专用 launch：只启动 传感器 + 底盘 + 静态 TF，用于 ros2 bag record 母数据包。
# 刻意不启动：EKF(ekf_filter_node) / slam_toolbox / dual_laser_merger / Nav2 / voice_nav / rviz。
#   - /scan_merged 可由母包离线重算，录制时无需融合节点；
#   - odom→base_link、map→odom 等动态 TF 由回放时的 EKF/SLAM 重新生成，
#     因此录制时 /tf 预期为空（录上是作漏网检测用），静态外参走 /tf_static。
# 配套录制命令（另开终端）：
#   ros2 bag record -o wheelchair_raw_test_01 /scan_1 /scan_2 /imu /odom /cmd_vel /tf_static /tf

def generate_launch_description():
    # Launch 参数
    use_sim_time = LaunchConfiguration('use_sim_time')

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

    # 2) IMU 的静态 TF（z=0.2，与 double_nav2.launch.py 一致；如需改回 0 只改这里）
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher_imu',
        arguments=['0.185', '0.0', '0.2', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
    )

    # 3) 激光雷达驱动启动（双 LSN10P → /scan_1、/scan_2）
    laser_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lslidar_driver'),
                'launch',
                'lsn10p_double_launch.py'
            ])
        )
    )

    # 4) fdilink_ahrs 驱动（→ /imu）
    imu_driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fdilink_ahrs'),
                'launch',
                'ahrs_driver.launch.py'
            ])
        )
    )

    # 5) 里程计（底盘驱动节点，只发 /odom 话题、订阅 /cmd_vel，TF 广播已在源码中删除）
    odom_node = Node(
        package='dsw_chair',
        executable='dsw_chair',
        name='odom_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ====== 返回 LaunchDescription ======
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true'),

        # 执行顺序：先起传感器，再挂 TF，最后底盘
        laser_node,
        imu_driver_node,
        static_tf_node_1,
        static_tf_node_2,
        static_tf_imu,
        odom_node,
    ])
