import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("lio_sam_hesai")
    parameter_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config_file = os.path.join(share_dir, "config", "rviz2.rviz")

    params_declare = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(share_dir, "config", "relocalization.yaml"),
        description="Full path to the ROS2 parameters file to use.",
    )
    use_sim_time_declare = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true.",
    )
    colorized_output = SetEnvironmentVariable(
        "RCUTILS_COLORIZED_OUTPUT",
        "1",
    )

    nodes = [
        # base_link -> hesai_lidar
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0.02", "0.0", "0.32", "0.0", "0.0", "0.0",
                "base_link",
                "hesai_lidar",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
            emulate_tty=True,
        ),
        # base_link -> imu_link
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0.02", "0.0", "0.02", "0.0", "0.0", "0.0",
                "base_link",
                "imu_link",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="lio_sam_hesai",
            executable="lio_sam_hesai_relocalization",
            name="lio_sam_hesai_relocalization",
            parameters=[parameter_file, {"use_sim_time": use_sim_time}],
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
            emulate_tty=True,
        ),
    ]

    return LaunchDescription(
        [params_declare, use_sim_time_declare, colorized_output] + nodes
    )
