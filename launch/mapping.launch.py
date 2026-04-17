import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("lio_sam_hesai")
    parameter_file = LaunchConfiguration("params_file")
    rviz_config_file = os.path.join(share_dir, "config", "rviz2.rviz")

    params_declare = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(share_dir, "config", "mapping.yaml"),
        description="Full path to the ROS2 parameters file to use.",
    )

    nodes = [
        # map -> odom (static in mapping mode)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen",
        ),
        # # base_link -> hesai_lidar
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     arguments=[
        #         "0.02", "0.0", "0.32", "0.0", "0.0", "0.0",
        #         "base_link",
        #         "hesai_lidar",
        #     ],
        #     output="screen",
        # ),
        # # base_link -> imu_link
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     arguments=[
        #         "0.02", "0.0", "0.02", "0.0", "0.0", "0.0",
        #         "base_link",
        #         "imu_link",
        #     ],
        #     output="screen",
        # ),
        Node(
            package="lio_sam_hesai",
            executable="lio_sam_hesai_imuPreintegration",
            parameters=[parameter_file],
            output="screen",
        ),
        Node(
            package="lio_sam_hesai",
            executable="lio_sam_hesai_imageProjection",
            name="lio_sam_hesai_imageProjection",
            parameters=[parameter_file],
            output="screen",
        ),
        Node(
            package="lio_sam_hesai",
            executable="lio_sam_hesai_featureExtraction",
            name="lio_sam_hesai_featureExtraction",
            parameters=[parameter_file],
            output="screen",
        ),
        Node(
            package="lio_sam_hesai",
            executable="lio_sam_hesai_mapOptimization",
            name="lio_sam_hesai_mapOptimization",
            parameters=[parameter_file],
            output="screen",
            sigterm_timeout="30",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen",
            emulate_tty=True,
        ),
    ]

    return LaunchDescription([params_declare] + nodes)
