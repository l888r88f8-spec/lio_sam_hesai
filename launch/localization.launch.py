import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, Substitution
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


class _RelocalizationGdbPrefix(Substitution):
    """Build the GDB prefix only when debug mode is enabled.

    The GDB log file lives in launch's configured log directory. That directory
    is created and owned by launch, so this substitution must not create it. The
    path is resolved lazily: with debug disabled neither the log directory nor
    HOME is touched, which keeps default launches working with a read-only HOME.
    """

    def __init__(self, debug):
        super().__init__()
        self.__debug = debug
        self.__prefix = None

    def perform(self, context):
        if self.__prefix is not None:
            return self.__prefix
        if str(self.__debug.perform(context)).lower() != "true":
            return ""

        gdb_log_file = os.path.join(
            launch_config.log_dir,
            "lio_sam_hesai_relocalization_gdb_"
            f"{time.strftime('%Y%m%d-%H%M%S')}_{os.getpid()}_"
            f"{time.time_ns()}.log",
        )
        self.__prefix = (
            "gdb -q --batch "
            '-ex "set pagination off" '
            '-ex "set confirm off" '
            f'-ex "set logging file {gdb_log_file}" '
            '-ex "set logging overwrite off" '
            '-ex "set logging enabled on" '
            "-ex run "
            '-ex "thread apply all bt full" '
            '-ex "set logging enabled off" '
            "--args"
        )
        return self.__prefix


def generate_launch_description():
    share_dir = get_package_share_directory("lio_sam_hesai")
    parameter_file = LaunchConfiguration("params_file")
    use_sim = LaunchConfiguration("use_sim")
    publish_static_tf = LaunchConfiguration("publish_static_tf")
    debug_relocalization = LaunchConfiguration("debug_relocalization")
    rviz_config_file = os.path.join(share_dir, "config", "rviz2.rviz")
    relocalization_prefix = _RelocalizationGdbPrefix(
        debug_relocalization
    )

    params_declare = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(share_dir, "config", "relocalization.yaml"),
        description="Full path to the ROS2 parameters file to use.",
    )
    use_sim_declare = DeclareLaunchArgument(
        "use_sim",
        default_value=os.environ.get("USE_SIM_TIME") or "true",
        description=(
            "Use simulation clock if true. Defaults to USE_SIM_TIME from the "
            "launch process environment, or true when unset; an explicit "
            "use_sim:=... overrides it."
        ),
    )
    publish_static_tf_declare = DeclareLaunchArgument(
        "publish_static_tf",
        default_value="true",
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
            parameters=[{"use_sim_time": use_sim}],
            output="screen",
            emulate_tty=True,
            condition=IfCondition(publish_static_tf),
        ),
        # base_link -> imu_link
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0.02", "0.0", "0.22", "0.0", "0.0", "0.0",
                "base_link",
                "imu_link",
            ],
            parameters=[{"use_sim_time": use_sim}],
            output="screen",
            emulate_tty=True,
            condition=IfCondition(publish_static_tf),
        ),
        Node(
            package="lio_sam_hesai",
            executable="lio_sam_hesai_relocalization",
            name="lio_sam_hesai_relocalization",
            parameters=[parameter_file, {"use_sim_time": use_sim}],
            output="both",
            emulate_tty=True,
            prefix=relocalization_prefix,
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            parameters=[{"use_sim_time": use_sim}],
            output="screen",
            emulate_tty=True,
        ),
    ]

    return LaunchDescription(
        [
            params_declare,
            use_sim_declare,
            publish_static_tf_declare,
            debug_relocalization_declare,
            colorized_output,
        ]
        + nodes
    )
