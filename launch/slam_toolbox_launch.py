
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
    slam_params_file = LaunchConfiguration('slam_params_file')

    current_dir=get_package_share_directory('simple_navigation')
    rviz_config_dir = os.path.join(current_dir,'rviz','slam_rviz2_config.rviz')
    labview_inter_dir=get_package_share_directory('labview_r2interface')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(current_dir,'param', 'mapping_settings.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    #run rviz2 with settings
    run_rviz2=Node(
                    package='rviz2',
                   executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_dir],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen')

        #launch the labview interface programs
    launch_labviewinterface=IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(labview_inter_dir,'launch', 'r2interface.launch.py')))

    ld = LaunchDescription()


    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(launch_labviewinterface)
    ld.add_action(run_rviz2)

    return ld
