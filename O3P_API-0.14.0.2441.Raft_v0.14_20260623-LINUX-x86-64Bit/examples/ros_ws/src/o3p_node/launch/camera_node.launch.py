from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_no = LaunchConfiguration('serial_no')
    camera_name = LaunchConfiguration('camera_name')
    recording = LaunchConfiguration('recording')
    loop = LaunchConfiguration('loop')
    use_timestamps = LaunchConfiguration('use_timestamps')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_no',
            default_value='',
            description='Serial number of the camera to open. Empty selects the first available camera.',
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='camera',
            description='Name used as prefix for topics and TF frames so several cameras can be distinguished.',
        ),
        DeclareLaunchArgument(
            'recording',
            default_value='',
            description='Path to a recording (.bag) to play back instead of opening a live camera. '
                        'Empty uses a live camera.',
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='true',
            description='Whether to loop the playback once the end of the recording is reached.',
        ),
        DeclareLaunchArgument(
            'use_timestamps',
            default_value='true',
            description='Whether to delay frame delivery according to the recorded timestamps.',
        ),
        Node(
            package='o3p_node',
            executable='camera_node',
            name='o3p_camera',
            output='screen',
            parameters=[{
                'serial_no': serial_no,
                'camera_name': camera_name,
                'recording': recording,
                'loop': loop,
                'use_timestamps': use_timestamps,
            }],
        ),
    ])
