import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'dr_spaam_ros2'
    JRDB_bag_path = os.path.expanduser('~/Documents/JRDB/train_dataset_with_activity/rosbags/')
    bag_path = os.path.expanduser('~/JACKAL_FILES/glenn_bags/')
    # Get configuration directory
    config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'params.yaml'
    )

    # Launch argument, whether to use Sick or VLP-16 LiDAR
    lidar_arg = DeclareLaunchArgument(
        'use_sick',
        default_value='false',
        description='Use Sick LiDAR if true'
    )
    lidar_arg_val = LaunchConfiguration('use_sick')

    # Launch argument for bag file
    bag_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='test5',
        description='Bag file to play'
    )
    bag_arg_val = LaunchConfiguration('bag_file')

    dr_spaam_node = Node(
        package=package_name,
        executable='dr_spaam_ros2_node',
        name='dr_spaam_ros2_node',
        output='screen',
        parameters=[config_path]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.expanduser('~/JACKAL_FILES/jackal.rviz'),
                   '--ros-args', '-r', '/tf:=/j100_0565/tf', '-r', '/tf_static:=/j100_0565/tf_static']
    )

    velodyne_node = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        name='velodyne_laserscan_node',
        output='screen',
        remappings=[
            ('velodyne_points', '/velodyne_points'),
            ('scan', '/scan')
        ],
        parameters=[{
            'ring': 8,
            'resolution': 0.00347, #0.00576
            # 'calibration': os.path.join(get_package_share_directory('velodyne_laserscan'), 'params', 'VLP16db.yaml'),
            # 'min_range': 0.3,
            # 'max_range': 100.0,
            # 'view_direction': 0.0,
            # 'view_width': 360.0,
            # 'organize_cloud': True,
            # 'scan_mode': 'Dual Return',
            # 'frame_id': 'velodyne',
            # 'model': 'VLP16'
        }],
        condition=UnlessCondition(lidar_arg_val)  # Only launch if use_sick is false
    )

    bag_publisher =ExecuteProcess(
        cmd=['ros2', 'bag', 'play', [bag_path, bag_arg_val], 
             '--topics', 
             '/j100_0565/sensors/camera_0/color/raw/compressed',
             '/j100_0565/sensors/camera_0/color/raw/camera_info',
             #'/j100_0565/sensors/lidar3d_0/scan', 
             '/j100_0565/sensors/lidar3d_0/points',
             '/j100_0565/tf', '/j100_0565/tf_static',
            '--remap', 
             '/j100_0565/sensors/camera_0/color/raw/compressed:=/camera/image/compressed',
             '/j100_0565/sensors/camera_0/color/raw/camera_info:=/j100_0565/sensors/camera_0/color/camera_info',
             #'/j100_0565/sensors/lidar3d_0/scan:=/scan', 
             '/j100_0565/sensors/lidar3d_0/points:=/velodyne_points',
             '--loop'],
        output='screen',
        condition=UnlessCondition(lidar_arg_val)  # Only launch if use_sick is false
    )

    return LaunchDescription([
        bag_arg,
        lidar_arg,
        dr_spaam_node,
        rviz_node,
        velodyne_node,
        #+vlp_laser_publisher,
        bag_publisher
    ])
