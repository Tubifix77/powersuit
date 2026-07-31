#include "suit_canspi_bridge/audio_relay.hpp"

#include <algorithm>
#include <cstring>

extern "C" {
#include "powersuit_proto/can_id.h"
#include "powersuit_proto/wire.h"
}

namespace suit_canspi_bridge {

AudioRelay::AudioRelay(rclcpp::Node* node) : node_(node) {
  ps_adpcm_init(&decode_state_);
  ps_adpcm_init(&encode_state_);

  uplink_pub_ = node_->create_publisher<suit_msgs::msg::AudioChunk>("/suit/audio/uplink", 20);
  downlink_sub_ = node_->create_subscription<suit_msgs::msg::AudioChunk>(
      "/suit/audio/downlink", 20,
      [this](const suit_msgs::msg::AudioChunk::SharedPtr msg) { on_downlink(msg); });
}

void AudioRelay::handle_uplink_record(const ps_can_record_t& rec) {
  ps_can_id_t parts{};
  ps_can_id_unpack(rec.id, &parts);
  if (parts.cls != PS_CLS_AUDIO) {
    return;
  }

  if (parts.type == PS_T_AUDIO_SYNC) {
    ps_audio_sync_t sync{};
    PS_WIRE_READ(sync, rec.data);
    if (sync.dir == 1) {  // uplink resync: adopt predictor/step_index
      decode_state_.predictor = sync.predictor;
      decode_state_.step_index = sync.step_index;
    }
    return;
  }
  if (parts.type != PS_T_AUDIO_UP) {
    return;
  }

  const size_t nbytes = rec.dlc;
  if (nbytes == 0) {
    return;
  }
  std::vector<int16_t> pcm(nbytes * 2);
  const size_t n = ps_adpcm_decode(&decode_state_, rec.data, nbytes, pcm.data());

  suit_msgs::msg::AudioChunk msg;
  msg.header.stamp = node_->get_clock()->now();
  msg.node_id = parts.src;
  msg.codec = suit_msgs::msg::AudioChunk::CODEC_PCM16;
  msg.sample_rate = 8000;
  msg.seq = uplink_seq_++;
  msg.data.resize(n * 2);
  std::memcpy(msg.data.data(), pcm.data(), n * 2);
  uplink_pub_->publish(msg);
}

void AudioRelay::on_downlink(const suit_msgs::msg::AudioChunk::SharedPtr msg) {
  if (msg->codec == suit_msgs::msg::AudioChunk::CODEC_ADPCM8K) {
    // Already ADPCM (e.g. re-encoded upstream by the voice/gateway nodes):
    // just re-frame it onto the wire, no shadow codec work needed.
    process_adpcm_down(msg->data.data(), msg->data.size());
    return;
  }
  if (msg->codec == suit_msgs::msg::AudioChunk::CODEC_PCM16) {
    const size_t n_samples = msg->data.size() / 2;
    const size_t n_even = n_samples - (n_samples % 2);  // ps_adpcm_encode requires even n
    if (n_even == 0) {
      return;
    }
    std::vector<int16_t> pcm(n_even);
    std::memcpy(pcm.data(), msg->data.data(), n_even * 2);
    std::vector<uint8_t> adpcm(n_even / 2);
    const size_t nbytes = ps_adpcm_encode(&encode_state_, pcm.data(), n_even, adpcm.data());
    process_adpcm_down(adpcm.data(), nbytes);
    return;
  }
  RCLCPP_WARN(node_->get_logger(), "audio_relay: unknown downlink codec %u",
              static_cast<unsigned>(msg->codec));
}

void AudioRelay::process_adpcm_down(const uint8_t* bytes, size_t n) {
  size_t off = 0;
  while (off < n) {
    if (frames_since_sync_ == 0) {
      maybe_enqueue_sync();
    }
    const size_t chunk = std::min<size_t>(8, n - off);
    enqueue_frame_down(bytes + off, chunk);
    off += chunk;
    ++down_frame_seq_;
    frames_since_sync_ = (frames_since_sync_ + 1) % kAudioSyncEveryFrames;
  }
}

void AudioRelay::enqueue_frame_down(const uint8_t* adpcm, size_t len) {
  ps_can_record_t rec{};
  rec.id = ps_can_id_pack(PS_CLS_AUDIO, PS_NODE_ORCH, PS_NODE_HELMET, PS_T_AUDIO_DOWN,
                           static_cast<uint8_t>(down_frame_seq_ & 0xFFu));
  rec.bus = PS_SPI_BUS_CAN1;  // helmet is on CAN 1
  rec.dlc = static_cast<uint8_t>(len);
  rec.ts_ms = 0;
  std::memset(rec.data, 0, sizeof(rec.data));
  std::memcpy(rec.data, adpcm, len);

  std::lock_guard<std::mutex> lock(mu_);
  outbound_.push_back(rec);
}

void AudioRelay::maybe_enqueue_sync() {
  ps_audio_sync_t sync{};
  sync.dir = 0;  // down
  sync.step_index = static_cast<uint8_t>(encode_state_.step_index);
  sync.predictor = static_cast<int16_t>(encode_state_.predictor);
  sync.frame_seq = down_frame_seq_;
  sync.rsvd = 0;

  ps_can_record_t rec{};
  rec.id = ps_can_id_pack(PS_CLS_AUDIO, PS_NODE_ORCH, PS_NODE_HELMET, PS_T_AUDIO_SYNC, 0);
  rec.bus = PS_SPI_BUS_CAN1;
  rec.dlc = sizeof(sync);
  rec.ts_ms = 0;
  std::memset(rec.data, 0, sizeof(rec.data));
  PS_WIRE_WRITE(rec.data, sync);

  std::lock_guard<std::mutex> lock(mu_);
  outbound_.push_back(rec);
}

std::vector<ps_can_record_t> AudioRelay::drain_outbound() {
  std::lock_guard<std::mutex> lock(mu_);
  std::vector<ps_can_record_t> out(outbound_.begin(), outbound_.end());
  outbound_.clear();
  return out;
}

}  // namespace suit_canspi_bridge
