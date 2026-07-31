# Battery status synthesized from TELEM BMS_SUMMARY (0x25) + BMS_CELLS (0x26).
# fault_bits per docs/network-map.md §4.4.
std_msgs/Header header
float32 pack_v
float32 current_a
uint8 soc_pct
int8 temp_max_c
uint16 fault_bits
float32[] cell_v     # per-cell volts, rotating groups of 3 accumulated
