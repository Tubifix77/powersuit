// Voice plane relay (docs/network-map.md §3.5, §8). Uplink FRAME_UP/SYNC
// records are decoded with a shadow ADPCM decoder (ps_adpcm_*) and
// republished as suit_msgs/AudioChunk codec=CODEC_PCM16 so no downstream
// consumer ever has to track ADPCM codec state. Downlink accepts either
// codec (ADPCM passthrough-reframe, or PCM16 encoded here) and emits
// FRAME_DOWN records with a SYNC every 50 frames.
#ifndef SUIT_CANSPI_BRIDGE_AUDIO_RELAY_HPP_
#define SUIT_CANSPI_BRIDGE_AUDIO_RELAY_HPP_

#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <suit_msgs/msg/audio_chunk.hpp>

extern "C" {
#include "powersuit_proto/spi_frame.h"
#include "powersuit_proto/adpcm.h"
}

namespace suit_canspi_bridge {

constexpr int kAudioSyncEveryFrames = 50;

class AudioRelay {
public:
  explicit AudioRelay(rclcpp::Node* node);

  // Dispatch one parsed AUDIO-class CanRecord (FRAME_UP or SYNC dir=up).
  void handle_uplink_record(const ps_can_record_t& rec);

  // Records generated from downlink AudioChunk callbacks, ready to enqueue
  // into the next outbound SPI frame.
  std::vector<ps_can_record_t> drain_outbound();

private:
  void on_downlink(const suit_msgs::msg::AudioChunk::SharedPtr msg);
  void process_adpcm_down(const uint8_t* bytes, size_t n);
  void enqueue_frame_down(const uint8_t* adpcm, size_t len);
  void maybe_enqueue_sync();

  rclcpp::Node* node_;
  rclcpp::Publisher<suit_msgs::msg::AudioChunk>::SharedPtr uplink_pub_;
  rclcpp::Subscription<suit_msgs::msg::AudioChunk>::SharedPtr downlink_sub_;

  ps_adpcm_state_t decode_state_{};
  ps_adpcm_state_t encode_state_{};
  uint32_t uplink_seq_ = 0;
  uint16_t down_frame_seq_ = 0;
  int frames_since_sync_ = 0;

  std::mutex mu_;
  std::deque<ps_can_record_t> outbound_;
};

}  // namespace suit_canspi_bridge

#endif  // SUIT_CANSPI_BRIDGE_AUDIO_RELAY_HPP_
