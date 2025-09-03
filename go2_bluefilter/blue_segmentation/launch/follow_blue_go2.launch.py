# follow_blue_go2.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    img = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/camera/color/image_raw/compressed'
    )
    mode = DeclareLaunchArgument(
        'go2_mode', default_value='0'  # 0: StandUp
    )

    return LaunchDescription([
        img, mode,

        Node(
            package='blue_segmentation',
            executable='blue_filter_node',
            name='blue_filter',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'use_compressed': True,
                'h_min': 90, 'h_max': 140,
                's_min': 50, 'v_min': 50,
                'kernel_size': 5,
                'publish_overlay': True,
                'log_fps': True,
                'save_segmented_image': True,
            }],
            output='screen'
        ),
        Node(
            package='blue_segmentation',
            executable='blue_follower_node',
            name='blue_follower',
            parameters=[{
                'target_topic': '/blue_mask',
                'cmd_vel_topic': '/cmd_vel',
                'center_threshold': 0.4,
                'linear_speed': 0.30,
                'angular_speed': 0.50,
            }],
            output='screen'
        ),
        # C++ 브리지: /cmd_vel -> Unitree Move()
        Node(
            package='unitree_ros2_example',
            executable='go2_sport_client',
            name='go2_cmdvel_bridge',
            arguments=[LaunchConfiguration('go2_mode')],
            remappings=[('lf/sportmodestate', '/sportmodestate')],
            output='screen'
        ),
    ])
