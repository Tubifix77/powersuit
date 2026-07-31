"""Bring up the orchestrator against tools/suit_sim instead of real hardware.

Identical to suit.launch.py except the bridge uses its mock transport, which
speaks the same 512-byte SPI framing over TCP. Start the simulator first:

    python -m suit_sim.serve --port 9700

The PTYs and the micro-ROS agent are still real — only the wire below the
bridge is simulated.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from suit_bringup.launch_common import XRCE_DIR, XRCE_NODES


def generate_launch_description() -> LaunchDescription:
    params = PathJoinSubstitution([FindPackageShare("suit_bringup"), "config", "params.yaml"])
    host = LaunchConfiguration("mock_host")
    port = LaunchConfiguration("mock_port")

    bridge = Node(
        package="suit_canspi_bridge",
        executable="suit_canspi_bridge",
        name="suit_canspi_bridge",
        parameters=[params, {"transport": "mock", "mock_host": host, "mock_port": port}],
        output="screen",
        emulate_tty=True,
    )

    agent = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=["micro-ros-agent", "multiserial", "--devs",
                     " ".join(f"{XRCE_DIR}/{n}" for n in XRCE_NODES), "-v4"],
                output="screen",
            )
        ],
    )

    estimation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("suit_state_estimation"), "launch",
                                  "ekf.launch.py"])
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("mock_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("mock_port", default_value="9700"),
        bridge,
        agent,
        estimation,
        Node(package="suit_ik", executable="ik_node", parameters=[params], output="screen"),
        Node(package="suit_ik", executable="flight_mapper", parameters=[params], output="screen"),
        Node(package="suit_voice_local", executable="voice_node", parameters=[params],
             output="screen"),
        Node(package="suit_cloud_gateway", executable="gateway_node", parameters=[params],
             output="screen"),
        Node(package="suit_blackbox", executable="blackbox_node", parameters=[params],
             output="screen"),
    ])
