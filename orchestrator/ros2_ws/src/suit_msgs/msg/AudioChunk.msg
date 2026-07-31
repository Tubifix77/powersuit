# Voice audio chunk (docs/network-map.md §3.5, §8).
# codec values mirror docs/link-protocol.md §3.
uint8 CODEC_ADPCM8K=1
uint8 CODEC_PCM16=2

std_msgs/Header header
uint8 node_id        # originating/destination edge node (6 = helmet)
uint8 codec          # CODEC_*
uint16 sample_rate   # Hz (8000 for the suit voice plane)
uint32 seq           # per-stream chunk sequence
uint8[] data         # codec payload (ADPCM bytes or PCM16LE bytes)
