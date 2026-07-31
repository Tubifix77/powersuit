"""State estimation stack: aggregator + IMU selector + robot_state_publisher + EKF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("suit_state_estimation")
    urdf_xacro = os.path.join(share, "urdf", "powersuit.urdf.xacro")
    ekf_yaml = os.path.join(share, "config", "ekf.yaml")

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", urdf_xacro]), value_type=str)

    return LaunchDescription([
        Node(
            package="suit_state_estimation",
            executable="joint_state_aggregator",
            name="suit_joint_state_aggregator",
            output="screen",
        ),
        Node(
            package="suit_state_estimation",
            executable="imu_selector",
            name="suit_imu_selector",
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            parameters=[ekf_yaml],
            output="screen",
        ),
    ])
