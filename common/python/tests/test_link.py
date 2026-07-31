import pytest

from powersuit_proto import link


def test_envelope_roundtrip():
    env = link.make_envelope(link.T_VOICE_QUERY, 12, {"query_id": "q-1", "text": "status report"})
    back = link.decode_envelope(link.encode_envelope(env))
    assert back == env


@pytest.mark.parametrize(
    "raw",
    [
        "not json",
        "[1,2,3]",
        '{"v":2,"type":"hello","seq":1,"ts":0,"payload":{}}',
        '{"v":1,"seq":1,"ts":0,"payload":{}}',
        '{"v":1,"type":"hello","seq":-1,"ts":0,"payload":{}}',
        '{"v":1,"type":"hello","seq":1,"payload":{}}',
        '{"v":1,"type":"hello","seq":1,"ts":0,"payload":[]}',
    ],
)
def test_envelope_rejects_malformed(raw):
    with pytest.raises(link.LinkProtocolError):
        link.decode_envelope(raw)


def test_audio_frame_roundtrip():
    payload = bytes(range(64))
    frame = link.pack_audio_frame(link.BTYPE_TTS_CHUNK, link.CODEC_ADPCM_8K, 7, 4242, payload)
    btype, codec, stream_id, seq, out = link.parse_audio_frame(frame)
    assert (btype, codec, stream_id, seq, out) == (
        link.BTYPE_TTS_CHUNK, link.CODEC_ADPCM_8K, 7, 4242, payload,
    )


def test_audio_frame_size_limits():
    with pytest.raises(link.LinkProtocolError):
        link.pack_audio_frame(1, 1, 0, 0, b"x" * (link.MAX_BIN_PAYLOAD + 1))
    with pytest.raises(link.LinkProtocolError):
        link.parse_audio_frame(b"\x01\x01\x00")  # shorter than header


def test_downlink_whitelist_is_closed():
    # The whitelist must never quietly grow an actuation path.
    assert link.DOWNLINK_WHITELIST == {
        link.T_HELLO_ACK, link.T_ADVISORY, link.T_TTS_META,
        link.T_AUDIO_CREDIT, link.T_ERROR, link.T_BYE,
    }
    assert link.T_TELEMETRY_BATCH not in link.DOWNLINK_WHITELIST
    assert link.T_VOICE_QUERY not in link.DOWNLINK_WHITELIST
