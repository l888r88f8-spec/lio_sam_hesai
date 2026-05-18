import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("lio_sam_hesai")
    parameter_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    publish_static_tf = LaunchConfiguration("publish_static_tf")
    debug_relocalization = LaunchConfiguration("debug_relocalization")
    rviz_config_file = os.path.join(share_dir, "config", "rviz2.rviz")
    gdb_log_file = os.path.join(
        os.path.expanduser("~"),
        ".ros",
        "log",
        "lio_sam_hesai_relocalization_gdb.log",
    )
    relocalization_prefix = PythonExpression(
        [
            "'gdb -q --batch "
            "-ex \"set pagination off\" "
            "-ex \"set confirm off\" "
            "-ex \"set logging file ",
            gdb_log_file,
            "\" "
            "-ex \"set logging overwrite on\" "
            "-ex \"set logging enabled on\" "
            "-ex run "
            "-ex \"thread apply all bt full\" "
            "-ex \"set logging enabled off\" "
            "--args' if '",
            debug_relocalization,
            "'.lower() == 'true' else ''",
        ]
    )

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
        default_value="false",
        description="Launch RViz if true.",
    )
    publish_static_tf_declare = DeclareLaunchArgument(
        "publish_static_tf",
        default_value="false",
        description="Publish base_link->lidar and base_link->imu static transforms.",
    )
    debug_relocalization_declare = DeclareLaunchArgument(
        "debug_relocalization",
        default_value="false",
        description="Run the relocalization node under GDB and print a full backtrace if it crashes.",
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
            condition=IfCondition(publish_static_tf),
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
            condition=IfCondition(publish_static_tf),
        ),
        Node(
            package="lio_sam_hesai",
            executable="lio_sam_hesai_relocalization",
            name="lio_sam_hesai_relocalization",
            parameters=[parameter_file, {"use_sim_time": use_sim_time}],
            output="screen",
            emulate_tty=True,
            prefix=relocalization_prefix,
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
            debug_relocalization_declare,
            colorized_output,
        ]
        + nodes
    )
