import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():

    # MAIN PARAMETERS TO CHANGE HERE
    map_name = "turtlebot3_world.yaml"
    # map_name= 'fake_map.yaml'
    # map_name= 'dmro_lab_7jun.yaml'
    # map_name= 'myfirstmap.yaml'
    # param_name='nav_config.yaml'

    param_name = "office_bot.yaml"
    # remember to add back the default one
    #    bt_filename='navigate_distance_replan.xml'
    bt_filename = "navigate_replanning_recovery.xml"
    use_sim_time = LaunchConfiguration("use_sim_time")
    map = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    # for libopenvdb to work
    set_ev = SetEnvironmentVariable(
        "LD_PRELOAD", "/usr/lib/x86_64-linux-gnu/libjemalloc.so.2"
    )
    current_dir = get_package_share_directory("simple_navigation")
    rviz_config_dir = os.path.join(current_dir, "rviz", "ot_bot_rviz.rviz")
    labview_inter_dir = get_package_share_directory("labview_r2interface")
    office_bot_des_dir = get_package_share_directory("officebot_description")
    twist_mux_dir = get_package_share_directory("cmd_vel_mux")
    bt_filename_dir = os.path.join(current_dir, "behaviour_trees", bt_filename)
    realsense_ros_dir = get_package_share_directory("realsense2_camera")

    ld = LaunchDescription()

    declare_map = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(current_dir, "map", map_name),
        description="Full path to map file to load",
    )

    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(current_dir, "param", param_name),
        description="Full path to param file to load",
    )

    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    # launch navigation
    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(current_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "default_bt_xml_filename": bt_filename_dir,
        }.items(),
    )
    # launch the labview interface programs
    launch_labviewinterface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(labview_inter_dir, "launch", "r2interface.launch.py")
        )
    )
    # launch robot state publisher
    launch_officebot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(office_bot_des_dir, "launch", "bot_des.launch.py")
        )
    )
    # launch twist mux
    launch_twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(twist_mux_dir, "launch", "cmd_vel_mux-launch.py")
        )
    )
    launch_realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_ros_dir, "launch", "rs_launch.py")
        ),
        launch_arguments={"enable_pointcloud": "true", "device_type": "d435"}.items(),
    )

    # run rviz2 with settings
    run_rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_dir],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    ld.add_action(declare_map)
    ld.add_action(declare_params)
    ld.add_action(declare_sim_time)
    ld.add_action(launch_navigation)
    ld.add_action(run_rviz2)
    #   ld.add_action(launch_labviewinterface)
    #   ld.add_action(launch_officebot_description)
    ld.add_action(launch_twist_mux)
    ld.add_action(set_ev)
    #  ld.add_action(launch_realsense_camera)

    return ld
