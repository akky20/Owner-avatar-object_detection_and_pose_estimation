from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    serial_no = LaunchConfiguration('serial_no')
    json_file_path = LaunchConfiguration('json_file_path')
    camera = LaunchConfiguration('camera')
    resolution = LaunchConfiguration('resolution')
    object_prefix = LaunchConfiguration('object_prefix')
    objects_path = LaunchConfiguration('objects_path')
    gui = LaunchConfiguration('gui')
    settings_path = LaunchConfiguration('settings_path')

    declare_arguments = [
        DeclareLaunchArgument('serial_no', default_value='', description='Camera serial number'),
        DeclareLaunchArgument('json_file_path', default_value='', description='Path to JSON configuration file'),
        DeclareLaunchArgument('camera', default_value='camera', description='Camera namespace'),
        DeclareLaunchArgument('resolution', default_value='qhd', description='Camera resolution'),
        DeclareLaunchArgument('object_prefix', default_value='object', description='Prefix for detected objects'),
        DeclareLaunchArgument('objects_path', default_value='', description='Path to object database'),
        DeclareLaunchArgument('gui', default_value='true', description='Enable GUI for find_object_2d'),
        DeclareLaunchArgument('settings_path', default_value='~/.ros/find_object_2d.ini', description='Path to settings file'),
    ]

    # Include realsense2_camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'serial_no': serial_no,
            'json_file_path': json_file_path,
            'camera_namespace': camera,
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
           'enable_depth': 'true',
            'enable_color': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'pointcloud.enable': 'true',
            # 'align_depth.enable': 'true',
            'enable_sync': 'true',
            'tf_prefix': camera,
        }.items()
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=camera,
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('realsense2_camera'),
                'examples',
                'pointcloud',
                'rviz',
                'pointcloud.rviz'
        ])],
        output='screen',
    )

    # find_object_2d node
    find_object_2d_node = Node(
        package='find_object_2d',
        executable='find_object_2d',
        name='find_object_2d',
        output='screen',
        parameters=[
            {'gui': LaunchConfiguration('gui')},
            {'settings_path': LaunchConfiguration('settings_path')},
            {'subscribe_depth': True},
            {'objects_path': LaunchConfiguration('objects_path')},
            {'object_prefix': LaunchConfiguration('object_prefix')},
            {'approx_sync': False},
            {
                'session_path': PathJoinSubstitution([
                    FindPackageShare('pointcloud_object_detection'),
                    'sessions',
                    'session1.bin'
                ])
            },
        ],
        remappings=[
            ('rgb/image_rect_color', f'/camera/camera/color/image_raw'),
            ('depth_registered/image_raw', f'/camera/camera/depth/image_rect_raw'),
            ('depth_registered/camera_info', f'/camera/camera/depth/camera_info'),
        ],
    )

    # tf_example node
    tf_example_node = Node(
        package='find_object_2d',
        executable='tf_example',
        name='tf_example',
        output='screen',
        parameters=[
            {'map_frame_id': f'/{camera}'},
            {'object_prefix': LaunchConfiguration('object_prefix')},
        ],
    )

    return LaunchDescription(
        declare_arguments + [
            realsense_launch,
            rviz_node,
            find_object_2d_node,
            tf_example_node,
        ]
    )
