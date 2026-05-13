import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("lio_sam_hesai")
    parameter_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    publish_static_tf = LaunchConfiguration("publish_static_tf")
    rviz_config_file = os.path.join(share_dir, "config", "rviz2.rviz")

    params_declare = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(share_dir, "config", "relocalization.yaml"),
        description="Full path to the shared relocalization + ground segmentation parameters file to use.",
    )
    use_sim_time_declare = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true.",
    )
    use_rviz_declare = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz if true.",
    )
    publish_static_tf_declare = DeclareLaunchArgument(
        "publish_static_tf",
        default_value="true",
        description="Publish base_link->lidar and base_link->imu static transforms.",
    )
    colorized_output = SetEnvironmentVariable(
        "RCUTILS_COLORIZED_OUTPUT",
        "1",
    )

    nodes = [
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
            condition=IfCondition(publish_static_tf),
        ),
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
            condition=IfCondition(publish_static_tf),
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
            package="lio_sam_hesai",
            executable="lio_sam_hesai_imageProjection",
            name="lio_sam_hesai_ground_segmentation",
            parameters=[
                parameter_file,
                {
                    "segmentation_only_mode": True,
                    "use_sim_time": use_sim_time,
                },
            ],
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
            condition=IfCondition(use_rviz),
        ),
    ]

    return LaunchDescription(
        [
            params_declare,
            use_sim_time_declare,
            use_rviz_declare,
            publish_static_tf_declare,
            colorized_output,
        ]
        + nodes
    )
