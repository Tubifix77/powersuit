"""Bring up the whole orchestrator on real hardware.

Order matters in one place: the micro-ROS agent must be attached to the PTYs the
bridge creates, so the bridge starts first and the agent follows after a short
delay. Everything else is order-independent — the suit's safety behaviour does
not depend on any of these nodes being up, by design.
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

    transport = LaunchConfiguration("transport")
    spi_device = LaunchConfiguration("spi_device")

    bridge = Node(
        package="suit_canspi_bridge",
        executable="suit_canspi_bridge",
        name="suit_canspi_bridge",
        parameters=[params, {"transport": transport, "device": spi_device}],
        output="screen",
        emulate_tty=True,
    )

    # One agent process serves every edge node: the bridge terminates each
    # client's HDLC stream on its own PTY, which is exactly what the serial
    # transport expects (docs/network-map.md §7).
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
        DeclareLaunchArgument("transport", default_value="spidev",
                              description="spidev on the suit, mock against tools/suit_sim"),
        DeclareLaunchArgument("spi_device", default_value="/dev/spidev0.0"),
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
