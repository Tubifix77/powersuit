# Cloud link state (docs/link-protocol.md §5, §6).
std_msgs/Header header
string bearer        # active bearer name ("5g", "wifi", "sat", "" when none)
float32 rtt_ms
float32 loss_pct
uint32 switches      # bearer switches since start
uint32 rejected      # downlink messages dropped by the whitelist
bool connected
