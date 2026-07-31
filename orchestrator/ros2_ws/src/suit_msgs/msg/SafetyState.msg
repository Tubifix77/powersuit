# Suit safety state as seen by the Node 8 bridge (docs/safety.md §2).
# state values mirror PS_STATE_* / powersuit_proto.wire.State.
uint8 STATE_BOOT=0
uint8 STATE_STANDBY=1
uint8 STATE_OPERATIONAL=2
uint8 STATE_PASSIVE=3
uint8 STATE_ESTOP=4
uint8 STATE_FAULT=5

std_msgs/Header header
uint8 state
bool estop_latched
bool heartbeat_ok    # bridge heartbeat gate currently open and emitting
uint8 estop_cause    # PS_ESTOP_* cause of the latched estop (0 when none)
string source        # who latched it ("operator", "node_5", "bridge", ...)
