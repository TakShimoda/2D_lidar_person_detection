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
    JRDB_bag_ros2_path = os.path.expanduser('~/Documents/JRDB/ros2bags/')
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
        default_value='bytes-cafe-2019-02-07_0', #'memorial-court-2019-03-16_0',
        description='Bag file to play'
    )
    bag_arg_val = LaunchConfiguration('bag_file')

    # base_link to laser transform
    tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
    )

    # velodyne to base_link transform
    tf_publisher2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'velodyne', 'base_link']
    )

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
        arguments=['-d', os.path.expanduser('~/Documents/VPT_ws/install/dr_spaam_ros2/share/dr_spaam_ros2/config/dr_spaam.rviz')]
                   #'--ros-args', '-r', '/tf:=/j100_0565/tf', '/tf_static:=/j100_0565/tf_static']
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
            'resolution': 0.00576
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

    sick_laser_publisher = ExecuteProcess(
        cmd=['python3', 'jrdb_bag_scan_publisher.py', [JRDB_bag_path, bag_arg_val, '.bag'], '--camera', '0', '--loop'],
        output='screen',
        cwd=os.path.expanduser('~/Documents/2D_lidar_person_detection/tools/'),
        condition=IfCondition(lidar_arg_val)  # Only launch if use_sick is true
    )

    vlp_laser_publisher = ExecuteProcess(
        cmd=['python3', 'jrdb_scan_publisher.py', '--mode', 'pointcloud', '--loop', '--sequence', bag_arg_val],
        output='screen',
        cwd=os.path.expanduser('~/Documents/2D_lidar_person_detection/tools/'),
        condition=UnlessCondition(lidar_arg_val)  # Only launch if use_sick is false
    )

    img_publisher =ExecuteProcess(
        cmd=['ros2', 'bag', 'play', [JRDB_bag_ros2_path, bag_arg_val], '--remap' ,'/ros_indigosdk_node/image0/compressed:=/camera/image/compressed', '/lower_velodyne/velodyne_points:=/velodyne_points', '--loop'],#, '--remap', 'ros_indigosdk_node/image0/compressed:=/camera/image'],
        output='screen',
        condition=UnlessCondition(lidar_arg_val)  # Only launch if use_sick is false
    )

    return LaunchDescription([
        bag_arg,
        lidar_arg,
        tf_publisher,
        tf_publisher2,
        dr_spaam_node,
        rviz_node,
        velodyne_node,
        sick_laser_publisher,
        #vlp_laser_publisher,
        img_publisher
    ])
