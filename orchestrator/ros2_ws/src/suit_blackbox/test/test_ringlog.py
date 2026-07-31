"""Native tests for suit_blackbox.ringlog — ROS-free."""

from suit_blackbox.ringlog import (
    HEADER_SIZE,
    RECORD_SIZE,
    RingLog,
    read_snapshot,
)


def _ring(tmp_path, capacity):
    return RingLog(tmp_path / "ring.bin", HEADER_SIZE + capacity * RECORD_SIZE)


def test_sizes_locked():
    assert HEADER_SIZE == 64
    assert RECORD_SIZE == 64


def test_append_and_order_before_wrap(tmp_path):
    with _ring(tmp_path, 8) as ring:
        for i in range(5):
            assert ring.append(float(i), i, (i * 1.5,), f"t{i}")
        recs = list(ring.records())
        assert len(recs) == 5 == len(ring)
        assert [r.code for r in recs] == [0, 1, 2, 3, 4]
        assert recs[3].vals[0] == 4.5
        assert recs[3].vals[5] == 0.0
        assert recs[2].tag == b"t2"


def test_wrap_around_order(tmp_path):
    with _ring(tmp_path, 4) as ring:
        for i in range(11):  # capacity 4, wraps twice
            ring.append(float(i), i, (float(i),), f"r{i}")
        assert ring.write_index == 11
        assert len(ring) == 4
        recs = list(ring.records())
        assert [r.code for r in recs] == [7, 8, 9, 10]  # oldest -> newest
        assert [r.ts for r in recs] == [7.0, 8.0, 9.0, 10.0]


def test_snapshot_correctness(tmp_path):
    with _ring(tmp_path, 4) as ring:
        for i in range(6):
            ring.append(float(i), i, (float(i), -float(i)), f"s{i}")
        out = tmp_path / "snap.bin"
        n = ring.snapshot(out)
        assert n == 4
        snap = read_snapshot(out)
        assert [r.code for r in snap] == [2, 3, 4, 5]
        assert snap[0].vals[1] == -2.0
        assert snap[-1].tag == b"s5"
        # snapshot is a copy: appending afterwards must not change it
        ring.append(99.0, 99, (), "later")
        assert [r.code for r in read_snapshot(out)] == [2, 3, 4, 5]


def test_freeze_idempotence(tmp_path):
    with _ring(tmp_path, 4) as ring:
        ring.append(1.0, 1, (), "a")
        assert ring.freeze() is True      # first freeze marks
        assert ring.frozen
        assert ring.freeze() is False     # second freeze is a no-op
        assert ring.frozen
        assert ring.append(2.0, 2, (), "b") is False  # frozen: no writes
        assert len(ring) == 1
        ring.thaw()
        assert ring.append(3.0, 3, (), "c") is True
        assert [r.code for r in ring.records()] == [1, 3]


def test_persistence_across_reopen(tmp_path):
    path = tmp_path / "ring.bin"
    size = HEADER_SIZE + 4 * RECORD_SIZE
    with RingLog(path, size) as ring:
        for i in range(6):
            ring.append(float(i), i, (), f"p{i}")
    with RingLog(path, size) as ring:  # same geometry -> state restored
        assert ring.write_index == 6
        assert [r.code for r in ring.records()] == [2, 3, 4, 5]


def test_tag_truncated_to_22_bytes(tmp_path):
    with _ring(tmp_path, 2) as ring:
        ring.append(0.0, 7, (), "x" * 40)
        rec = next(iter(ring.records()))
        assert rec.tag == b"x" * 22
