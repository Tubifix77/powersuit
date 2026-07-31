"""Constants shared by the launch files.

XRCE_NODES must match the symlink names created by
suit_canspi_bridge/src/xrce_demux.cpp — the agent attaches to those paths, so a
mismatch here shows up as six silently dead micro-ROS sessions.
"""

XRCE_DIR = "/tmp/powersuit/xrce"

XRCE_NODES = (
    "node_arm_right",
    "node_arm_left",
    "node_leg_right",
    "node_leg_left",
    "node_flight_actuation",
    "node_helmet_interface",
)
