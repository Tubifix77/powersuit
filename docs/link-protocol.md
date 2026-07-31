# Suit ↔ Cloud Link Protocol (Node 8 ↔ Node 9)

**Normative.** Transport: TLS WebSockets. Control messages are JSON text frames; audio is
binary frames. A protobuf mirror (`proto/suit_link.proto`) exists for a future gRPC path and is
kept schema-identical but is optional and never on the test path. Reference codecs:
`cloud/cloud_ai_core/protocol.py` (server) and `orchestrator/.../suit_cloud_gateway` (client);
`tools/suit_sim` exercises both.

## 1. Connection lifecycle

1. Client (Node 8 gateway) connects to `wss://<endpoint>/link` over the currently selected
   bearer (§5). No credentials in the URL, ever.
2. First frame MUST be `hello` (text). Server answers `hello_ack` or closes with code 4001
   (auth failed) / 4002 (version unsupported).
3. After `hello_ack`, both sides may send any message type. WS protocol-level ping/pong
   (not a JSON type) measures liveness/RTT; client pings every 5 s, 3 misses ⇒ reconnect.
4. Clean shutdown: `bye` then WS close 1000.

Reconnect/resume: the client presents `session_id` + `last_rx_seq` in `hello.resume`; if the
server still holds the session (TTL 120 s) it replays queued downlink messages after
`resume_from` — bearer failover (5G→Wi-Fi→Sat) is lossless for control messages. Audio is NOT
replayed (stale voice is worse than silence); the TTS stream restarts at the next sentence.

## 2. JSON envelope (text frames)

```json
{"v": 1, "type": "<string>", "seq": <u64>, "ts": <float unix seconds>, "payload": {…}}
```

- `seq` is per-direction, monotonically increasing from 1.
- Unknown `type` ⇒ respond `error` (code `unknown_type`), do not close.
- All timestamps are sender-local Unix time; skew is measured via `hello_ack.ts_echo`.

### 2.1 `hello` (client → server)

```json
{"v":1,"type":"hello","seq":1,"ts":…,"payload":{
  "proto": 1,
  "suit_id": "powersuit-01",
  "token": "<bearer token>",
  "agent": "suit_cloud_gateway/0.1",
  "resume": {"session_id": "…", "last_rx_seq": 418}   // optional
}}
```

### 2.2 `hello_ack` (server → client)

```json
{"payload":{
  "session_id": "b3f0…",
  "resume_from": 0,            // server's first seq for this session epoch
  "ts_echo": 1753…,            // client's hello ts, echoed
  "audio_credits": 32,         // initial TTS flow-control credits (binary frames)
  "heartbeat_s": 5,
  "capabilities": ["rag","tts","advisory"]
}}
```

### 2.3 `telemetry_batch` (client → server, coalesced at 5 Hz)

```json
{"payload":{
  "window_ms": 200,
  "sources": {
    "arm_right":  {"joints":[{"j":0,"pos":1.21,"vel":0.02,"eff":3.4},…], "imu_ok":true},
    "leg_left":   {…},
    "torso_imu":  {"quat":[1,0,0,0],"acc":[0,0,-9.8]},
    "power":      {"pack_v":50.2,"current_a":12.1,"soc":81,"t_max":41,"faults":0},
    "aero":       {"ias":0.0,"q":0.0,"flaps":[0,0,…]},
    "safety":     {"state":"OPERATIONAL","estop":false,"hb_ok":true},
    "env":        {"temp_c":24.1}
  },
  "dropped_count": 0          // TELEM frames lost since last batch (gap vs calm discriminator)
}}
```
Units on the wire here are SI floats (the cloud never sees packed centi-units).

### 2.4 `voice_query` (client → server)

```json
{"payload":{"query_id":"q-17","text":"status report","source":"helmet",
            "context":{"state":"OPERATIONAL","soc":81}}}
```
`text` is produced by Node 8 (local ASR or keyword path). If the suit streams raw audio up
instead, binary AUDIO_UP frames carry it (§3) and the server's ASR fills `text` server-side;
the `voice_query` then references `stream_id`.

### 2.5 `advisory` (server → client)

```json
{"payload":{"advisory_id":"a-99","severity":"info|notice|warning|critical",
            "title":"Battery pacing","body":"Cell group 3 running 4°C above…",
            "hud":{"icon":"battery","ttl_s":10},          // optional HUD suggestion
            "in_reply_to":"q-17"}}                        // optional
```

### 2.6 `tts_meta` (server → client)  — announces/ends a TTS stream

```json
{"payload":{"stream_id": 7, "query_id":"q-17", "state":"start|end|abort",
            "codec":"adpcm8k", "text":"Suit nominal. Battery eighty-one percent."}}
```

### 2.7 `audio_credit` (client → server) — flow control

```json
{"payload":{"stream_id": 7, "credits": 16}}   // grants N more binary frames
```
Server MUST stop sending when credits hit 0. Client grants in blocks as its downlink
buffer drains toward the helmet. Advisories are never credit-gated.

### 2.8 `link_stats` (client → server, 0.2 Hz) / `error` / `bye`

`link_stats.payload`: `{"bearer":"wifi","rtt_ms":38,"loss_pct":0.2,"switches":1}`.
`error.payload`: `{"code":"unknown_type|bad_payload|auth|internal","detail":"…","ref_seq":123}`.
`bye.payload`: `{"reason":"shutdown|failover|idle"}`.

## 3. Binary frames (audio)

All WS binary frames carry an 8-byte header, little-endian, then payload:

```
0: u8  btype   0x01 = TTS_CHUNK (server→client), 0x02 = AUDIO_UP_CHUNK (client→server)
1: u8  codec   0x01 = IMA ADPCM 8 kHz mono, 0x02 = PCM16LE 16 kHz mono
2: u16 stream_id
4: u32 seq     per-stream, from 0
8: …   payload (ADPCM: multiple of 8 bytes; ≤ 2048 bytes per frame)
```

TTS_CHUNK payloads are consumed against `audio_credits` (one frame = one credit).
ADPCM state resets at each stream start; server inserts decoder-state sync every 50×8-byte
blocks in-band via the suit's AUDIO SYNC frames after Node 8 transcoding (the WS payload itself
is continuous ADPCM; Node 8 re-frames it for CAN descent).

## 4. Authentication & transport security

- TLS 1.3; server certificate verified against the CA bundle in `links.yaml`
  (self-signed CA supported for bench setups; pinning optional per endpoint).
- Bearer token in `hello.payload.token` only (never URL query, never HTTP header logs).
  Tokens come from `POWERSUIT_LINK_TOKEN` env / systemd credential on Node 8 and
  `CLOUD_LINK_TOKENS` (comma-separated) on Node 9.
- 4001 close on bad token; client backs off (1 s → 32 s exponential, jittered).

## 5. Bearer selection (Node 8 `link_monitor`)

`links.yaml` lists endpoints with `priority` (5g=1, wifi=2, sat=3 by default — lower wins),
each probed by TCP connect + WS ping when idle-candidate. Scoring: healthy = connected AND
rtt_ms < bearer budget (5g 150 ms, wifi 80 ms, sat 1200 ms) AND loss < 5 %. Switch when the
active bearer is unhealthy for 3 s OR a higher-priority bearer is healthy for 10 s
(hysteresis, no flapping). On switch: keep old socket until `hello_ack` on the new one
(make-before-break), then resume (§1).

## 6. Downlink whitelist (hard security boundary)

Node 8's gateway accepts ONLY these server-originated types: `hello_ack`, `advisory`,
`tts_meta`, `audio_credit`(echo), `error`, `bye`, binary `TTS_CHUNK`. Everything else is
dropped and counted (`link_stats.rejected`). Advisories may *suggest* HUD content; they are
rendered by the HUD composer with a `[CLOUD]` provenance tag and never touch
`/suit/command/*`, FLAP_CMD, MODE_SET, or the SAFETY plane. There is deliberately no message
type by which the cloud can actuate the suit; adding one requires changing this document, the
whitelist, and `suit_sim` scenario 10, which exists to prove local safety action outruns any
cloud round-trip.

## 7. Server-side pipeline (Node 9)

`voice_query` → RAG retrieve (top-k over `docs/manuals/`) → LLM engine (OpenAI-compatible:
vLLM in production, Ollama/LM Studio for bench, deterministic mock in tests) → response text →
(a) `advisory` if informational, and/or (b) TTS stream: synthesize → resample 8 kHz mono →
IMA ADPCM encode (same codec as `common/`, locked by shared vectors) → `tts_meta start` →
credit-gated TTS_CHUNKs → `tts_meta end`. Telemetry batches update the rolling suit state;
threshold rules (`telemetry/rules.py`) emit unsolicited advisories (e.g. cell overtemp trend).
