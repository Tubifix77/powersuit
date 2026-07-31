"""Native tests for suit_voice_local.matcher — ROS-free."""

from pathlib import Path

import pytest

from suit_voice_local.matcher import FALLBACK_CONFIDENCE, Matcher, normalize
from suit_voice_local.tts_stub import BEEP_PATTERNS, SAMPLE_RATE, synth_ack

GRAMMAR = Path(__file__).resolve().parents[1] / "config" / "grammar.yaml"


@pytest.fixture(scope="module")
def matcher():
    return Matcher.from_file(GRAMMAR)


def test_normalize_strips_punctuation():
    assert normalize("Deploy the AIR-brakes, now!") == ["deploy", "the", "air", "brakes", "now"]
    assert normalize("brightness 0.5.") == ["brightness", "0.5"]


@pytest.mark.parametrize("text,expected", [
    ("status report", "status_report"),
    ("give me a status", "status_report"),
    ("systems check", "status_report"),
    ("deploy airbrakes", "deploy_airbrakes"),
    ("full airbrakes now", "deploy_airbrakes"),
    ("retract airbrakes", "retract_airbrakes"),
    ("stow airbrakes", "retract_airbrakes"),
    ("power level", "power_level"),
    ("battery", "power_level"),
    ("emergency stop", "engage_estop"),
    ("estop", "engage_estop"),
    ("e-stop", "engage_estop"),
    ("clear estop", "clear_estop"),
    ("reset estop", "clear_estop"),
    ("hud brightness 80", "hud_brightness"),
])
def test_each_intent_matches(matcher, text, expected):
    intent, _slots, confidence = matcher.match(text)
    assert intent == expected, text
    assert confidence >= 0.5


def test_clear_beats_engage_on_longer_pattern(matcher):
    # "clear estop" contains "estop" (an engage pattern) — longest pattern must win.
    intent, _, _ = matcher.match("please clear the estop")
    assert intent == "clear_estop"


def test_fallback_is_cloud_query(matcher):
    intent, slots, confidence = matcher.match("what is the weather on approach")
    assert intent == "cloud_query"
    assert slots["text"] == "what is the weather on approach"
    assert confidence == FALLBACK_CONFIDENCE


def test_slot_parse_number(matcher):
    intent, slots, _ = matcher.match("set hud brightness to 42")
    assert intent == "hud_brightness"
    assert slots["value"] == 42.0
    intent, slots, _ = matcher.match("brightness 0.5")
    assert intent == "hud_brightness"
    assert slots["value"] == 0.5


def test_slot_absent_when_no_number(matcher):
    intent, slots, _ = matcher.match("hud brightness up")
    assert intent == "hud_brightness"
    assert "value" not in slots


def test_empty_input(matcher):
    intent, slots, confidence = matcher.match("   ")
    assert intent is None
    assert confidence == 0.0


def test_exact_command_confidence_is_full(matcher):
    _, _, confidence = matcher.match("deploy airbrakes")
    assert confidence == pytest.approx(1.0)


def test_ack_tones_distinct_and_pcm16():
    seen = set()
    for intent in BEEP_PATTERNS:
        pcm = synth_ack(intent, word_count=1)
        assert len(pcm) % 2 == 0 and len(pcm) > 0
        seen.add(pcm)
    assert len(seen) == len(BEEP_PATTERNS)  # per-intent distinct patterns


def test_ack_length_grows_with_words():
    short = synth_ack("status_report", word_count=1)
    long = synth_ack("status_report", word_count=6)
    assert len(long) > len(short)
    # sanity: at 8 kHz a 5-word bonus adds ~(30+200) ms = ~1840 samples
    assert (len(long) - len(short)) // 2 >= int(0.2 * SAMPLE_RATE)
