# Requirements:
#   A realsense D400 series
#   Install realsense2 ros2 package (refactor branch)
# Example:
#   $ ros2 run realsense_node realsense_node
#   $ ros2 launch rtabmap_ros realsense_d400.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'camera_link',
          'subscribe_depth':False,
          'subscribe_rgbd':False,
          'subscribe_stereo':True,
          'approx_sync':False}]

    remappings=[]

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        Node(
            package='rtabmap_ros', node_executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_ros', node_executable='rtabmapviz', output='screen',
            parameters=parameters,
            remappings=remappings),
    ])
