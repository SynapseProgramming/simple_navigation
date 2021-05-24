import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node




def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    map = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    current_dir=get_package_share_directory('simple_navigation')
    rviz_config_dir = os.path.join(current_dir,'rviz','main_nav2_test.rviz')

    #MAIN PARAMETERS TO CHANGE HERE
    #map_name='turtlebot3_world.yaml'
    map_name= 'myfirstmap.yaml'
    param_name='nav_config.yaml'


    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(current_dir,'map',map_name),
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(current_dir,'param',param_name),
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        #launch navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(current_dir,'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map,
                'use_sim_time': use_sim_time,
                'params_file': params_file}.items(),
        ),
        #run rviz2 with settings
        Node(
            package='rviz2',
           executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    ])
